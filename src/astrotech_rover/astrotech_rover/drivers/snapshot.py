"""Snapshot driver -- saves the current Raman / CO2 plot to disk on demand.

A Foxglove panel can't write files (it's sandboxed in the browser), so the
"save this plot" buttons in the Snapshots panel call ROS services that this
driver implements. It buffers the latest Raman spectrum and recent CO2
samples, and on a service call renders a PNG with matplotlib and writes it.

Where files go
--------------
First choice: ``~/Desktop/<desktop_subdir>/`` on the machine running this
node (run the node on the operator laptop to land on the laptop's Desktop).
Fallback: ``<repo>/<fallback_subdir>/`` if the Desktop isn't writable.
The save service returns the full path it wrote in its response ``message``.

This is always-on (not hardware-gated): it works the same whether the
Raman/env feeds are real or simulated. Started via :mod:`astrotech_node`'s
``_start`` so a missing matplotlib only disables snapshots, not the node.

Dependency: ``matplotlib`` (``python3-matplotlib`` rosdep, or pip install).
"""

from __future__ import annotations

import os
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

from rclpy.node import Node
from std_srvs.srv import Trigger

from cmr_msgs.msg import EnvSample, RamanSpectrum

# matplotlib is optional; import here so a clear message reaches the node's
# _start handler if it's missing, instead of a bare ImportError.
try:
    import matplotlib
    matplotlib.use("Agg")  # headless: render to file, no display needed
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
except ImportError as exc:  # pragma: no cover - depends on host install
    raise ImportError(
        "SnapshotDriver: matplotlib not installed. install with: "
        "sudo apt install python3-matplotlib  (or pip install matplotlib) "
        f"({exc})"
    ) from exc

# This file: src/astrotech_rover/astrotech_rover/drivers/snapshot.py
#   dirname + four ".." -> repo root (for the <repo>/snapshots fallback).
_REPO_ROOT = Path(
    os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                 "..", "..", "..", ".."))
)


class SnapshotDriver:
    def __init__(self, node: Node, cfg: dict) -> None:
        self._node = node
        snap = cfg["snapshot"]
        self._desktop_subdir = str(snap.get("desktop_subdir", "astrotech_snapshots"))
        self._fallback_subdir = str(snap.get("fallback_subdir", "snapshots"))
        self._co2_window_s = float(snap.get("co2_history_seconds", 120))

        self._lock = threading.Lock()
        self._latest_raman: RamanSpectrum | None = None
        # (monotonic_time, co2_ppm) ring buffer for the CO2 time-series plot.
        self._co2: deque[tuple[float, float]] = deque(maxlen=10000)

        node.create_subscription(
            RamanSpectrum, cfg["raman"]["topic"], self._on_raman, 10
        )
        node.create_subscription(
            EnvSample, cfg["env"]["topic"], self._on_env, 10
        )
        self._raman_srv = node.create_service(
            Trigger, snap["save_raman_service"], self._on_save_raman
        )
        self._co2_srv = node.create_service(
            Trigger, snap["save_co2_service"], self._on_save_co2
        )

        node.get_logger().info(
            f"SnapshotDriver up: {snap['save_raman_service']} / "
            f"{snap['save_co2_service']} -> ~/Desktop/{self._desktop_subdir} "
            f"(fallback {self._fallback_subdir}/)"
        )

    # --- subscriptions (executor threads) -------------------------------
    def _on_raman(self, msg: RamanSpectrum) -> None:
        with self._lock:
            self._latest_raman = msg

    def _on_env(self, msg: EnvSample) -> None:
        now = time.monotonic()
        with self._lock:
            self._co2.append((now, float(msg.co2_ppm)))
            cutoff = now - max(self._co2_window_s, 1.0) * 3.0  # keep some slack
            while self._co2 and self._co2[0][0] < cutoff:
                self._co2.popleft()

    # --- output path ----------------------------------------------------
    def _output_dir(self) -> Path:
        """~/Desktop/<subdir> if writable, else <repo>/<fallback>, else /tmp."""
        candidates = [
            Path.home() / "Desktop" / self._desktop_subdir,
            _REPO_ROOT / self._fallback_subdir,
            Path("/tmp") / self._desktop_subdir,
        ]
        for d in candidates:
            try:
                d.mkdir(parents=True, exist_ok=True)
                # confirm we can actually write here
                probe = d / ".write_test"
                probe.touch()
                probe.unlink()
                return d
            except Exception:  # noqa: BLE001 - try the next candidate
                continue
        # last resort: home dir
        return Path.home()

    def _save_figure(self, fig: Figure, kind: str) -> str:
        FigureCanvasAgg(fig)
        out_dir = self._output_dir()
        stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        path = out_dir / f"{stamp}_{kind}.png"
        fig.savefig(str(path), dpi=120, bbox_inches="tight")
        return str(path)

    # --- services -------------------------------------------------------
    def _on_save_raman(self, _req, resp: Trigger.Response) -> Trigger.Response:
        with self._lock:
            spec = self._latest_raman
        if spec is None or len(spec.intensities) == 0:
            resp.success = False
            resp.message = "no Raman spectrum received yet"
            return resp
        try:
            xs = list(spec.wavenumbers_cm_inv)
            ys = list(spec.intensities)
            have_x = len(xs) == len(ys)
            fig = Figure(figsize=(7, 4))
            ax = fig.add_subplot(111)
            ax.plot(xs if have_x else range(len(ys)), ys, lw=1.0)
            ax.set_xlabel("Raman shift (cm⁻¹)" if have_x else "pixel index")
            ax.set_ylabel("intensity")
            ax.set_title(f"Raman spectrum — {datetime.now():%Y-%m-%d %H:%M:%S}")
            ax.grid(True, alpha=0.3)
            path = self._save_figure(fig, "raman")
        except Exception as exc:  # noqa: BLE001 - report render/write failure
            resp.success = False
            resp.message = f"save failed: {exc}"
            self._node.get_logger().error(resp.message)
            return resp
        resp.success = True
        resp.message = path
        self._node.get_logger().info(f"saved Raman snapshot: {path}")
        return resp

    def _on_save_co2(self, _req, resp: Trigger.Response) -> Trigger.Response:
        with self._lock:
            data = list(self._co2)
        if not data:
            resp.success = False
            resp.message = "no CO2 samples received yet"
            return resp
        try:
            now = time.monotonic()
            # x = seconds before the snapshot (negative), within the window
            xs = [t - now for (t, _) in data if now - t <= self._co2_window_s]
            ys = [c for (t, c) in data if now - t <= self._co2_window_s]
            if not xs:
                xs = [t - now for (t, _) in data]
                ys = [c for (_, c) in data]
            fig = Figure(figsize=(7, 4))
            ax = fig.add_subplot(111)
            ax.plot(xs, ys, lw=1.0)
            ax.set_xlabel("seconds before snapshot")
            ax.set_ylabel("CO₂ (ppm)")
            ax.set_title(f"CO₂ — {datetime.now():%Y-%m-%d %H:%M:%S}")
            ax.grid(True, alpha=0.3)
            path = self._save_figure(fig, "co2")
        except Exception as exc:  # noqa: BLE001 - report render/write failure
            resp.success = False
            resp.message = f"save failed: {exc}"
            self._node.get_logger().error(resp.message)
            return resp
        resp.success = True
        resp.message = path
        self._node.get_logger().info(f"saved CO2 snapshot: {path}")
        return resp
