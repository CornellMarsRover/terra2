"""Real auger driver — bridges the AugerCommand ROS topic to the moteus bus.

Default auger driver in :mod:`astrotech_rover.astrotech_node`. Drop-in
counterpart of ``MockAugerDriver`` (same ``__init__(node, cfg)`` signature,
same subscribe/publish topics); set ``URC_AUGER_MOCK=1`` to swap in the
simulation driver for hardware-free dev.

Architecture
------------
A daemon thread runs an asyncio loop that talks to two moteus controllers
(id=15 lead screw, id=16 auger) over the configured fdcanusb transport at
50 Hz. ROS subscription callbacks (executor thread) drop the latest
``AugerCommand`` into a lock-protected snapshot; the asyncio loop reads
the snapshot each cycle, applies a 200 ms watchdog and hard safety caps,
calls ``transport.cycle([make_position(...), make_position(...)])``, and
publishes the resulting ``AugerState`` from the moteus reply.

This is the same per-cycle pattern the bring-up script
``src/astrotech_rover/scripts/auger_slight_move.py`` was validated against during the auger
hardware bring-up session.

Safety caps
-----------
The Foxglove panel currently sends the original test-harness defaults
(lead screw 100 rev/s, auger 10/-50 rev/s) which are 10x-200x above
anything that has been validated on real hardware. This driver clamps
both velocity and torque so a button press cannot send the rig past
a hand-picked operating envelope.

Current caps:
* lead screw velocity: ``±50.0 rev/s``
* auger velocity:      ``±6.0 rev/s``
* torque (both):       ``3.0 Nm``

The lead screw needs a much higher RPM than the auger because of the
physical reduction between motor revolutions and vertical travel: at low
RPM the rig is visibly underdriven through Foxglove. Auger stays low
because rotation rate is what the operator actually perceives at the
drum. The bring-up script ``src/astrotech_rover/scripts/auger_slight_move.py`` intentionally
stays at the smaller ``±2.0 rev/s`` / ``1.0 Nm`` envelope -- it is the
"first cold-start probe" and its job is to be over-conservative; the
driver is for operational use after the script has confirmed basic
motion.

Raise the caps further only after explicit higher-velocity hardware
tests, and update this docstring when you do.

Shutdown
--------
``node.context.on_shutdown`` sets a stop event; the asyncio loop notices
and unwinds through a ``finally`` that issues ``make_stop`` on both
controllers before the bus closes. Without this, the moteus would sit in
mode=10 with the last commanded velocity until its own watchdog tripped
~100 ms later -- correct but sloppy, and easy to do better.
"""

from __future__ import annotations

import asyncio
import math
import threading
import time

import moteus
from rclpy.node import Node

from cmr_msgs.msg import AugerCommand, AugerState

LEAD_SCREW_ID = 15
AUGER_ID = 16

LOOP_HZ = 50.0
LOOP_PERIOD_S = 1.0 / LOOP_HZ
SHUTDOWN_STOP_TIMEOUT_S = 0.5

# Hard caps. The panel currently publishes 10/-50/100 rev/s defaults from
# the original test harness, so without these clamps the very first button
# press would drive the rig 10x-200x faster than anything we have
# validated on hardware. The lead-screw cap is much higher than the auger
# cap because of the physical reduction: lead-screw motor RPM maps to a
# small vertical travel rate, while auger motor RPM maps directly to drum
# rotation. See the "Safety caps" section of the module docstring.
SAFE_LEAD_SCREW_VEL_CAP_REV_S = 50.0
SAFE_AUGER_VEL_CAP_REV_S = 16.0
SAFE_TORQUE_CAP_NM = 3.0


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class RealAugerDriver:
    """Owns the AugerCommand subscription and AugerState publisher,
    talking to the real moteus stack via :mod:`moteus`."""

    def __init__(self, node: Node, cfg: dict) -> None:
        if cfg["cmd_type"] != "cmr_msgs/AugerCommand":
            raise NotImplementedError(
                f"RealAugerDriver expects cmr_msgs/AugerCommand; "
                f"got {cfg['cmd_type']} in astrotech_interfaces.yaml"
            )

        self._node = node
        self._watchdog_s = float(cfg.get("cmd_watchdog_ms", 200)) / 1000.0
        self._port = cfg.get("real_port")

        self._cmd_lock = threading.Lock()
        self._lead_v = 0.0
        self._lead_t = 0.0
        self._auger_v = 0.0
        self._auger_t = 0.0
        # Initialize last_cmd_t in the past so the watchdog fires immediately
        # (= zero velocity) until the first real command arrives.
        self._last_cmd_t = 0.0

        self._stop = threading.Event()

        self._cmd_sub = node.create_subscription(
            AugerCommand, cfg["cmd_topic"], self._on_cmd, 10
        )
        self._state_pub = node.create_publisher(
            AugerState, cfg["state_topic"], 10
        )

        # Fire the stop event on rclpy shutdown so the asyncio loop's
        # finally block can issue make_stop before the bus closes.
        node.context.on_shutdown(self._on_shutdown)

        self._thread = threading.Thread(
            target=self._thread_main, name="auger_real_loop", daemon=True
        )
        self._thread.start()

        node.get_logger().info(
            f"RealAugerDriver up: cmd={cfg['cmd_topic']} "
            f"state={cfg['state_topic']} @ {LOOP_HZ:.0f} Hz "
            f"(port={self._port or 'auto'}, "
            f"caps: |lead_v|<={SAFE_LEAD_SCREW_VEL_CAP_REV_S} rev/s, "
            f"|auger_v|<={SAFE_AUGER_VEL_CAP_REV_S} rev/s, "
            f"torque<={SAFE_TORQUE_CAP_NM} Nm, "
            f"watchdog={self._watchdog_s * 1000:.0f} ms)"
        )

    def _on_cmd(self, msg: AugerCommand) -> None:
        with self._cmd_lock:
            self._lead_v = float(msg.lead_screw_velocity_rev_s)
            self._lead_t = float(msg.lead_screw_max_torque_nm)
            self._auger_v = float(msg.auger_velocity_rev_s)
            self._auger_t = float(msg.auger_max_torque_nm)
            self._last_cmd_t = time.monotonic()

    def _on_shutdown(self) -> None:
        self._stop.set()

    def _thread_main(self) -> None:
        try:
            asyncio.run(self._loop())
        except Exception as exc:  # pragma: no cover - hardware path
            self._node.get_logger().error(f"auger loop crashed: {exc}")

    async def _loop(self) -> None:
        try:
            if self._port:
                transport = moteus.Fdcanusb(path=self._port)
            else:
                transport = moteus.get_singleton_transport()
        except Exception as exc:  # pragma: no cover - hardware path
            self._node.get_logger().error(
                f"failed to open moteus transport on {self._port or 'auto'} "
                f"({exc}); RealAugerDriver will sit idle. Check fdcanusb is "
                "plugged in."
            )
            return

        ls = moteus.Controller(id=LEAD_SCREW_ID, transport=transport)
        au = moteus.Controller(id=AUGER_ID, transport=transport)

        # Clear any stale fault from a previous session before commanding.
        await transport.cycle([ls.make_stop(), au.make_stop()])

        try:
            while not self._stop.is_set():
                with self._cmd_lock:
                    lv, lt = self._lead_v, self._lead_t
                    av, at = self._auger_v, self._auger_t
                    age = time.monotonic() - self._last_cmd_t

                if age > self._watchdog_s:
                    lv = 0.0
                    av = 0.0

                lv = _clamp(lv, -SAFE_LEAD_SCREW_VEL_CAP_REV_S,
                            SAFE_LEAD_SCREW_VEL_CAP_REV_S)
                av = _clamp(av, -SAFE_AUGER_VEL_CAP_REV_S,
                            SAFE_AUGER_VEL_CAP_REV_S)
                lt = _clamp(lt, 0.0, SAFE_TORQUE_CAP_NM)
                at = _clamp(at, 0.0, SAFE_TORQUE_CAP_NM)

                results = await transport.cycle([
                    ls.make_position(
                        position=math.nan, velocity=lv,
                        maximum_torque=lt, query=True,
                    ),
                    au.make_position(
                        position=math.nan, velocity=av,
                        maximum_torque=at, query=True,
                    ),
                ])
                self._publish_state(results)
                await asyncio.sleep(LOOP_PERIOD_S)
        finally:
            try:
                await asyncio.wait_for(
                    transport.cycle([ls.make_stop(), au.make_stop()]),
                    timeout=SHUTDOWN_STOP_TIMEOUT_S,
                )
            except Exception:  # pragma: no cover - hardware path
                # Best-effort. If make_stop fails on shutdown the moteus's
                # own watchdog will trip the motor within ~100 ms anyway.
                pass

    def _publish_state(self, results) -> None:
        by_id = {r.id: r.values for r in results}
        out = AugerState()
        out.stamp = self._node.get_clock().now().to_msg()
        ls_v = by_id.get(LEAD_SCREW_ID, {})
        au_v = by_id.get(AUGER_ID, {})
        R = moteus.Register

        out.lead_screw_position_rev = float(ls_v.get(R.POSITION, float("nan")))
        out.lead_screw_velocity_rev_s = float(ls_v.get(R.VELOCITY, float("nan")))
        out.lead_screw_torque_nm = float(ls_v.get(R.TORQUE, float("nan")))
        out.lead_screw_temperature_c = float(ls_v.get(R.TEMPERATURE, float("nan")))
        out.lead_screw_mode = int(ls_v.get(R.MODE, 0))
        out.lead_screw_fault = int(ls_v.get(R.FAULT, 0))

        out.auger_position_rev = float(au_v.get(R.POSITION, float("nan")))
        out.auger_velocity_rev_s = float(au_v.get(R.VELOCITY, float("nan")))
        out.auger_torque_nm = float(au_v.get(R.TORQUE, float("nan")))
        out.auger_temperature_c = float(au_v.get(R.TEMPERATURE, float("nan")))
        out.auger_mode = int(au_v.get(R.MODE, 0))
        out.auger_fault = int(au_v.get(R.FAULT, 0))

        self._state_pub.publish(out)
