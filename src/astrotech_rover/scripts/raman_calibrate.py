#!/usr/bin/env python3
"""Fit the Raman pixel->wavelength calibration for astrotech_rover.

The real Raman driver (``drivers/raman_real.py``) turns a CCD pixel index into
a Raman shift (cm^-1) using two values under ``raman.real_calibration`` in
``config/astrotech_interfaces.yaml``::

    laser_nm                  the excitation laser wavelength (nm)
    pixel_to_wavelength_poly  [c0, c1, c2]  ASCENDING: nm = c0 + c1*p + c2*p^2

This script takes a few reference peaks whose true positions you know (from a
calibration lamp, or a known Raman standard such as silicon at 520.7 cm^-1),
fits pixel -> wavelength(nm), and prints the ``real_calibration`` YAML block
ready to paste -- already in the ascending order the driver expects.

Each ``--point`` is ``PIXEL:VALUE:UNIT`` where UNIT is ``cm-1`` (Raman shift)
or ``nm`` (absolute scattered wavelength). cm-1 points are converted to nm
with ``--laser-nm``. Use at least (order + 1) points; default fit is quadratic.

Example (silicon + two lamp lines, 785 nm laser)::

    python3 raman_calibrate.py --laser-nm 785 \\
        --point 410:520.7:cm-1 \\
        --point 1980:1332:cm-1 \\
        --point 3050:2900:cm-1

Stdlib only -- no numpy / no ROS needed, so it runs on any laptop.
"""
from __future__ import annotations

import argparse
import math
import sys


def shift_cm_inv_to_nm(laser_nm: float, shift: float) -> float:
    """Raman shift (cm^-1) -> absolute scattered wavelength (nm)."""
    inv = 1.0 / laser_nm - shift / 1.0e7
    if inv <= 0:
        raise SystemExit(
            f"shift {shift} cm^-1 is out of range for a {laser_nm} nm laser"
        )
    return 1.0 / inv


def nm_to_shift_cm_inv(laser_nm: float, wl_nm: float) -> float:
    """Absolute scattered wavelength (nm) -> Raman shift (cm^-1)."""
    return (1.0 / laser_nm - 1.0 / wl_nm) * 1.0e7


def parse_point(s: str, laser_nm: float) -> tuple[float, float]:
    parts = s.split(":")
    if len(parts) != 3:
        raise SystemExit(f"bad --point '{s}', expected PIXEL:VALUE:UNIT")
    try:
        pix = float(parts[0])
        val = float(parts[1])
    except ValueError:
        raise SystemExit(f"bad --point '{s}', PIXEL and VALUE must be numbers")
    unit = parts[2].strip().lower()
    if unit in ("nm", "wavelength"):
        return pix, val
    if unit in ("cm-1", "cm^-1", "cm_inv", "shift"):
        return pix, shift_cm_inv_to_nm(laser_nm, val)
    raise SystemExit(f"unknown unit '{unit}' in --point '{s}' (use nm or cm-1)")


def _solve(a: list[list[float]], b: list[float]) -> list[float]:
    """Solve a square linear system via Gaussian elimination w/ partial pivot."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-15:
            raise SystemExit(
                "calibration fit is singular -- reference pixels must be distinct."
            )
        m[col], m[piv] = m[piv], m[col]
        pv = m[col][col]
        for r in range(n):
            if r == col:
                continue
            f = m[r][col] / pv
            for c in range(col, n + 1):
                m[r][c] -= f * m[col][c]
    return [m[i][n] / m[i][i] for i in range(n)]


def _polymul(p: list[float], q: list[float]) -> list[float]:
    r = [0.0] * (len(p) + len(q) - 1)
    for i, pa in enumerate(p):
        for j, qb in enumerate(q):
            r[i + j] += pa * qb
    return r


def polyfit_ascending(xs: list[float], ys: list[float], order: int) -> list[float]:
    """Least-squares polynomial fit; returns ascending coeffs [c0, c1, ...].

    Fits in a centered/scaled variable u = (x - x0)/s (so the normal equations
    stay well-conditioned even for pixel indices in the thousands) and then
    expands back to coefficients in x.
    """
    n = len(xs)
    x0 = sum(xs) / n
    span = max(xs) - min(xs)
    s = span / 2.0 if span > 0 else 1.0
    us = [(x - x0) / s for x in xs]

    deg = order
    moments = [sum(u ** k for u in us) for k in range(2 * deg + 1)]
    a = [[moments[i + j] for j in range(deg + 1)] for i in range(deg + 1)]
    b = [sum(ys[i] * us[i] ** k for i in range(n)) for k in range(deg + 1)]
    coeffs_u = _solve(a, b)  # ascending coeffs in u

    # Expand sum_k coeffs_u[k] * ((x - x0)/s)^k into ascending coeffs in x.
    inner = [-x0 / s, 1.0 / s]  # u(x) = inner[0] + inner[1] * x
    result = [0.0]
    term = [1.0]  # inner^0
    for k in range(deg + 1):
        for i, c in enumerate(term):
            while len(result) <= i:
                result.append(0.0)
            result[i] += coeffs_u[k] * c
        term = _polymul(term, inner)
    return result


def polyval_ascending(coeffs_asc: list[float], x: float) -> float:
    v = 0.0
    for c in reversed(coeffs_asc):
        v = v * x + c
    return v


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Fit a Raman pixel->wavelength calibration polynomial."
    )
    ap.add_argument(
        "--laser-nm", type=float, required=True,
        help="excitation laser wavelength in nm (e.g. 785)",
    )
    ap.add_argument(
        "--point", action="append", default=[], metavar="PIXEL:VALUE:UNIT",
        help="reference peak; UNIT is cm-1 or nm. Repeatable.",
    )
    ap.add_argument(
        "--order", type=int, default=2,
        help="polynomial order (default 2 = quadratic)",
    )
    args = ap.parse_args(argv)

    if len(args.point) < args.order + 1:
        ap.error(
            f"need at least {args.order + 1} --point(s) for an order-{args.order} "
            f"fit (got {len(args.point)})"
        )

    pts = [parse_point(p, args.laser_nm) for p in args.point]
    pix = [p for p, _ in pts]
    wl = [w for _, w in pts]

    coeffs_asc = polyfit_ascending(pix, wl, args.order)

    # Report residuals in cm^-1 -- what the operator reads off the plot.
    res_cm = [
        nm_to_shift_cm_inv(args.laser_nm, polyval_ascending(coeffs_asc, p))
        - nm_to_shift_cm_inv(args.laser_nm, w)
        for p, w in pts
    ]
    rms_cm = math.sqrt(sum(r * r for r in res_cm) / len(res_cm)) if res_cm else 0.0

    print("# paste under `raman:` in config/astrotech_interfaces.yaml")
    print("  real_calibration:")
    print(f"    laser_nm: {args.laser_nm:g}")
    print(
        "    pixel_to_wavelength_poly: ["
        + ", ".join(f"{c:.10g}" for c in coeffs_asc)
        + "]"
    )
    print()
    print(f"# fit: order {args.order}, {len(pts)} points, RMS {rms_cm:.2f} cm^-1")
    for (p, w), r in zip(pts, res_cm):
        print(
            f"#   pixel {p:8.1f} -> {w:9.3f} nm "
            f"({nm_to_shift_cm_inv(args.laser_nm, w):8.1f} cm^-1)  "
            f"residual {r:+6.2f} cm^-1"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
