#!/usr/bin/env python3
"""Python-vs-C++ comparison for the drive core (run in the ROS docker image).

Feeds identical cases to the Python originals (cmr_rovernet) and to the C++
`swerve_targets_cli` test helper and requires bit-identical results for:
  * shape_motion_for_mode + calculate_swerve_targets (all four drive modes)
  * the node's _clamp
  * a random CommandMux operation trace (command_mux_core)

usage: compare_swerve_with_python.py <path/to/swerve_targets_cli | ->
           [--cases N] [--seed S] [--write-golden <file.inc>]
Needs `cmr_rovernet` importable (PYTHONPATH=src/cmr_rovernet) and a sourced
workspace that provides cmr_msgs (for the usama_control_testing import).
"""
import argparse
import math
import random
import struct
import subprocess
import sys

try:
    import tkinter  # noqa: F401  (moteus_drive_gui imports it at module level)
except Exception:  # headless image without tk: the GUI is not used here
    import types
    tk = types.ModuleType("tkinter")
    tk.ttk = types.ModuleType("tkinter.ttk")
    sys.modules["tkinter"] = tk
    sys.modules["tkinter.ttk"] = tk.ttk

from cmr_rovernet.command_mux_core import CommandMux
from cmr_rovernet.drive_modes import shape_motion_for_mode
from cmr_rovernet.usama_control_testing import (
    DRIVE_IDS,
    STEER_IDS,
    UsamaControlRosNode,
    calculate_swerve_targets,
)

MODES = ["translation_rotation", "ackermann", "steady_heading", "point_turn"]


def bits(x):
    if math.isnan(x):
        return "nan"
    return struct.pack("<d", x).hex()


def tok(x):
    if math.isnan(x):
        return "nan"
    if math.isinf(x):
        return "inf" if x > 0 else "-inf"
    return float(x).hex()


def parse(t):
    if t in ("nan", "inf", "-inf"):
        return float(t)
    return float.fromhex(t)


def swerve_cases(n, rng):
    cases = []
    grid = [-1.0, -0.5, -0.1, -0.0005, 0.0, 0.0005, 0.1, 0.5, 1.0]
    for mode in MODES:
        for vx in grid:
            for vy in grid:
                for om in grid:
                    cases.append((mode, vx, vy, om, 4.0, (0.0, 0.0, 0.0, 0.0)))
    # rounding ties / signed zero / speed extremes
    for v in (0.0005, 0.0015, 0.0025, -0.0005, -0.0015, 0.1235, 0.9995, -0.0, 1e-9):
        for mode in MODES:
            cases.append((mode, v, -v, v, -12.0, (0.25, -0.5, 1.75, -3.0)))
            cases.append((mode, -0.0, v, -0.0, 12.0, (0.0, 0.0, 0.0, 0.0)))
    for _ in range(n):
        mode = rng.choice(MODES)
        kind = rng.random()
        if kind < 0.6:
            v = [rng.uniform(-1, 1) for _ in range(3)]
        elif kind < 0.8:  # 3-decimal values (hit the round() boundaries)
            v = [rng.randint(-2000, 2000) / 2000.0 for _ in range(3)]
        else:  # axis-aligned commands, as an operator produces them
            v = [rng.choice([0.0, 0.0, rng.uniform(-1, 1)]) for _ in range(3)]
        rps = rng.choice([4.0, -4.0, 12.0, 2.0, 0.0, rng.uniform(-12, 12)])
        offs = tuple(rng.choice([0.0, rng.uniform(-25, 25)]) for _ in range(4))
        cases.append((mode, v[0], v[1], v[2], rps, offs))
    return cases


def py_swerve(case):
    mode, vx, vy, om, rps, offs = case
    m = shape_motion_for_mode(mode, vx, vy, om)
    t = calculate_swerve_targets(m[0], m[1], m[2], rps, dict(zip(STEER_IDS, offs)))
    return list(m) + [t["drive"][i] for i in DRIVE_IDS] + [t["steer"][i] for i in STEER_IDS]


def clamp_cases(rng):
    special = [0.0, -0.0, 1.0, -1.0, 1.5, -1.5, 12.0, -12.0, 1e-300, math.inf, -math.inf,
               math.nan]
    cases = [(v, lim) for v in special for lim in (1.0, 12.0, 0.0, 6.0)]
    cases += [(rng.uniform(-20, 20), rng.choice([1.0, 12.0, 4.0])) for _ in range(2000)]
    return cases


def mux_trace(rng, n):
    ops = [("new", 0.5)]
    now = 100.0
    srcs = ["teleop", "autonomy"]
    for _ in range(n):
        now += rng.choice([0.0, 0.01, 0.05, 0.1, 0.25, 0.5, 0.7, -0.05])
        r = rng.random()
        if r < 0.40:
            vals = [rng.choice([0.0, rng.uniform(-1, 1)]) for _ in range(4)]
            if rng.random() < 0.05:
                vals[rng.randrange(4)] = rng.choice([math.nan, math.inf, -math.inf])
            t = now if rng.random() > 0.02 else math.nan
            src = rng.choice(srcs) if rng.random() > 0.02 else "tank"
            ops.append(("recv", src, t, vals))
        elif r < 0.80:
            ops.append(("out", now if rng.random() > 0.02 else math.inf))
        elif r < 0.88:
            ops.append(("select", rng.choice(srcs + ["tank"])))
        elif r < 0.93:
            ops.append(("estop",))
        elif r < 0.98:
            ops.append(("reset",))
        else:
            ops.append(("new", rng.choice([0.5, 0.2, 0.0, -1.0, math.nan, math.inf])))
    return ops


def py_mux(ops):
    out = []
    mux = None
    for op in ops:
        try:
            if op[0] == "new":
                mux = CommandMux(op[1])
                out.append(["ok"])
            elif op[0] == "select":
                mux.select(op[1])
                out.append(["ok"])
            elif op[0] == "estop":
                mux.emergency_stop()
                out.append(["ok"])
            elif op[0] == "reset":
                mux.reset()
                out.append(["ok"])
            elif op[0] == "recv":
                out.append([str(mux.receive(op[1], tuple(op[3]), op[2]))])
            elif op[0] == "out":
                r = mux.output(op[1])
                out.append(["None"] if r is None else [bits(v) for v in r])
        except ValueError as exc:
            out.append(["ValueError"] + str(exc).split())
    return out


def mux_line(op):
    if op[0] == "new":
        return f"M new {tok(op[1])}"
    if op[0] == "select":
        return f"M select {op[1]}"
    if op[0] in ("estop", "reset"):
        return f"M {op[0]}"
    if op[0] == "recv":
        return f"M recv {op[1]} {tok(op[2])} " + " ".join(tok(v) for v in op[3])
    return f"M out {tok(op[1])}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cli")
    ap.add_argument("--cases", type=int, default=200000)
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--write-golden", default="")
    args = ap.parse_args()
    rng = random.Random(args.seed)

    sw = swerve_cases(args.cases, rng)
    cl = clamp_cases(rng)
    mx = mux_trace(rng, 20000)

    lines = []
    for mode, vx, vy, om, rps, offs in sw:
        lines.append(f"S {mode} {tok(vx)} {tok(vy)} {tok(om)} {tok(rps)} "
                     + " ".join(tok(o) for o in offs))
    lines += [f"C {tok(v)} {tok(lim)}" for v, lim in cl]
    lines += [mux_line(op) for op in mx]

    if args.cli == "-":  # golden generation only (no C++ helper built yet)
        got = None
    else:
        got = run_cli(args.cli, lines)
    bad = compare(sw, cl, mx, got) if got is not None else 0
    if args.write_golden:
        write_golden(args.write_golden)
    return 1 if bad else 0


def run_cli(cli, lines):
    res = subprocess.run([cli], input="\n".join(lines) + "\n", capture_output=True,
                         text=True, check=True)
    got = res.stdout.splitlines()
    assert len(got) == len(lines), (len(got), len(lines))
    return got


def compare(sw, cl, mx, got):
    bad = 0
    i = 0
    for case in sw:
        exp = [bits(v) for v in py_swerve(case)]
        cpp = [bits(parse(t)) for t in got[i].split()[1:]]
        if exp != cpp:
            bad += 1
            if bad <= 10:
                fields = [k for k, (a, b) in enumerate(zip(exp, cpp)) if a != b]
                print("SWERVE MISMATCH fields", fields, case, py_swerve(case), got[i])
        i += 1
    for v, lim in cl:
        exp = bits(UsamaControlRosNode._clamp(v, lim))
        cpp = bits(parse(got[i].split()[1]))
        if exp != cpp:
            bad += 1
            if bad <= 10:
                print("CLAMP MISMATCH", v, lim, UsamaControlRosNode._clamp(v, lim), got[i])
        i += 1
    for op, exp in zip(mx, py_mux(mx)):
        toks = got[i].split()[1:]
        cpp = toks if toks[0] in ("ok", "True", "False", "None", "ValueError") else [
            bits(parse(t)) for t in toks]
        if exp != cpp:
            bad += 1
            if bad <= 10:
                print("MUX MISMATCH", op, exp, got[i])
        i += 1

    print(f"compared: {len(sw)} swerve cases, {len(cl)} clamp cases, {len(mx)} mux ops; "
          f"mismatches: {bad}")
    return bad


def write_golden(path):
    if True:
        grng = random.Random(7)
        gold = swerve_cases(600, grng)[::23]
        with open(path, "w") as f:
            f.write("// Generated by test/compare_swerve_with_python.py --write-golden from the\n"
                    "// Python originals (drive_modes.shape_motion_for_mode +\n"
                    "// usama_control_testing.calculate_swerve_targets).  Do not edit.\n"
                    "// {mode, vx, vy, omega, drive_rps, {o5, o7, o6, o8},\n"
                    "//  {shaped vx, vy, omega, d1, d2, d3, d4, s5, s7, s6, s8}}\n")
            for case in gold:
                mode, vx, vy, om, rps, offs = case
                exp = py_swerve(case)
                f.write('{"%s", %s, %s, %s, %s, {%s}, {%s}},\n' % (
                    mode, tok(vx), tok(vy), tok(om), tok(rps),
                    ", ".join(tok(o) for o in offs), ", ".join(tok(v) for v in exp)))
        print(f"wrote {len(gold)} golden cases to {path}")


if __name__ == "__main__":
    sys.exit(main())
