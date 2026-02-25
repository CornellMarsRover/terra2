#!/usr/bin/env python3
import argparse
import asyncio
import math
import time
import socket
import json
import numpy as np
import moteus

DEBUG_NETWORK = True
DEBUG_COMMAND = True
DEBUG_MODULES = False

DEFAULT_CAN_PORT = "/dev/ttyACM0"
UDP_PORT = 5005

SWERVE_IDS = {
    "FL": (1, 5),
    "FR": (3, 7),
    "BL": (2, 6),
    "BR": (4, 8),
}

STEER_GEAR_RATIO = 50.0
DRIVE_GEAR_RATIO = 26.0

L = 0.8
W = 0.8
R = math.sqrt(L**2 + W**2)

MAX_V = 2
MAX_OMEGA = 1.0

ACCEL_RATE = 2.0
ROT_ACCEL_RATE = 2.0

DRIVE_MAX_TORQUE = 3.5
STEER_MAX_TORQUE = 3.0
STEER_VEL_LIMIT = 5.0

CONTROL_DT = 0.02
TELEMETRY_EVERY_N_TICKS = 10  # 5 Hz

DRIVE_SIGN = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}
STEER_SIGN = {"FL": 1, "FR": 1, "BL": 1, "BR": 1}

DZ = 0.08
UDP_TIMEOUT_S = 0.5

# When command magnitude is tiny, don't re-aim steers (prevents startup "rotate to 0")
IDLE_CMD_EPS = 0.03
# When wheel speed is tiny, keep steer angle
WHEEL_SPEED_EPS = 0.02


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def apply_deadzone(x: float, dz: float = DZ) -> float:
    return 0.0 if abs(x) < dz else x


def ramp(current: float, target: float, rate: float, dt: float) -> float:
    delta = target - current
    step = rate * dt
    return target if abs(delta) < step else current + math.copysign(step, delta)


def normalize_angle_rad(x: float) -> float:
    return (x + math.pi) % (2 * math.pi) - math.pi


def optimize_swerve(target_angle_rad: float, target_speed: float, current_angle_rad: float):
    diff = normalize_angle_rad(target_angle_rad - current_angle_rad)
    if abs(diff) > math.pi / 2:
        diff = normalize_angle_rad(diff - math.copysign(math.pi, diff))
        target_speed *= -1.0
    return current_angle_rad + diff, target_speed


def calculate_swerve(vx: float, vy: float, omega: float):
    v = np.zeros((2, 4), dtype=float)

    v[0, 0] = vx - omega * (L / R)  # FL x
    v[1, 0] = vy + omega * (W / R)  # FL y
    v[0, 1] = vx - omega * (L / R)  # FR x
    v[1, 1] = vy - omega * (W / R)  # FR y
    v[0, 2] = vx + omega * (L / R)  # BL x
    v[1, 2] = vy + omega * (W / R)  # BL y
    v[0, 3] = vx + omega * (L / R)  # BR x
    v[1, 3] = vy - omega * (W / R)  # BR y

    speeds = {
        "FL": math.hypot(v[0, 0], v[1, 0]),
        "FR": math.hypot(v[0, 1], v[1, 1]),
        "BL": math.hypot(v[0, 2], v[1, 2]),
        "BR": math.hypot(v[0, 3], v[1, 3]),
    }
    angles = {
        "FL": math.atan2(v[1, 0], v[0, 0]),
        "FR": math.atan2(v[1, 1], v[0, 1]),
        "BL": math.atan2(v[1, 2], v[0, 2]),
        "BR": math.atan2(v[1, 3], v[0, 3]),
    }

    max_s = max(speeds.values())
    if max_s > 1.0:
        for k in speeds:
            speeds[k] /= max_s

    return speeds, angles


def make_transport(serial_port: str):
    try:
        return moteus.Fdcanusb(serial_port)
    except TypeError:
        pass
    try:
        return moteus.Fdcanusb(port=serial_port)
    except TypeError:
        pass
    return moteus.Fdcanusb()


async def _call_with_no_reply_kw(callable_fn, **kwargs):
    for key in ("query", "query_cmd", "query_cmds"):
        try:
            return await callable_fn(**kwargs, **{key: False})
        except TypeError:
            pass
    return await callable_fn(**kwargs)


async def send_position_noreply(ctrl: moteus.Controller, **kwargs):
    return await _call_with_no_reply_kw(ctrl.set_position, **kwargs)


async def send_stop_noreply(ctrl: moteus.Controller, clear_faults: bool = False):
    if clear_faults:
        try:
            return await _call_with_no_reply_kw(ctrl.set_stop, clear_faults=True)
        except TypeError:
            pass
    return await _call_with_no_reply_kw(ctrl.set_stop)


async def safe_stop_all(ctrls, clear_faults: bool = False):
    await asyncio.gather(
        *[send_stop_noreply(c, clear_faults=clear_faults) for c in ctrls.values()],
        return_exceptions=True
    )


def motor_rot_to_wheel_rad(side: str, pos_rot: float) -> float:
    signed_rot = STEER_SIGN[side] * pos_rot
    return (signed_rot / STEER_GEAR_RATIO) * (2 * math.pi)


def wheel_rad_to_motor_rot(side: str, wheel_rad: float) -> float:
    motor_rot = (wheel_rad / (2 * math.pi)) * STEER_GEAR_RATIO
    return STEER_SIGN[side] * motor_rot


async def rezero_and_sync_steers(ctrls, steer_angle_est):
    # Stop outputs during rezero to avoid surprise motion
    await safe_stop_all(ctrls)

    # Rezero steer axes at current physical orientation
    await asyncio.gather(
        *[_call_with_no_reply_kw(ctrls[f"{s}_steer"].set_rezero) for s in SWERVE_IDS],
        return_exceptions=True
    )

    # Immediately query back and sync software estimate
    states = await asyncio.gather(
        *[ctrls[f"{s}_steer"].query() for s in SWERVE_IDS],
        return_exceptions=True
    )

    for side, st in zip(SWERVE_IDS.keys(), states):
        if isinstance(st, Exception) or st is None:
            print(f"[WARN] steer query failed for {side}: {st}")
            continue
        pos_rot = float(st.values.get(moteus.Register.POSITION, 0.0))
        steer_angle_est[side] = motor_rot_to_wheel_rad(side, pos_rot)

    print("[STARTUP] Steers rezeroed + synced.")


class NetworkInput:
    def __init__(self, port=UDP_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)

        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        self.last_packet_time = 0.0
        self.packet_count = 0
        print(f"[NET] Listening on UDP port {port}")

    def update(self):
        """Drain all available UDP packets; keep the most recent."""
        got = False
        try:
            while True:
                data, _addr = self.sock.recvfrom(2048)
                pkt = json.loads(data.decode(errors="ignore"))
                self.vx = float(pkt.get("vx", 0.0))
                self.vy = float(pkt.get("vy", 0.0))
                self.omega = float(pkt.get("omega", 0.0))
                self.last_packet_time = time.time()
                self.packet_count += 1
                got = True
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"[NET ERROR] {e}")

        # Timeout safety: if we stop receiving, commands should go idle
        if self.packet_count > 0 and (time.time() - self.last_packet_time) > UDP_TIMEOUT_S:
            self.vx = self.vy = self.omega = 0.0

        return got


async def main(serial_port: str):
    print("Initializing moteus transport...")
    transport = make_transport(serial_port)

    ctrls = {}
    for side, (drv_id, str_id) in SWERVE_IDS.items():
        ctrls[f"{side}_drive"] = moteus.Controller(id=drv_id, transport=transport)
        ctrls[f"{side}_steer"] = moteus.Controller(id=str_id, transport=transport)

    # Local steer angle estimate (wheel radians)
    steer_angle_est = {side: 0.0 for side in SWERVE_IDS}

    # Start safe (and clear faults once)
    await safe_stop_all(ctrls, clear_faults=True)
    await asyncio.sleep(0.05)

    # Rezero steers and sync estimate so first "hold" doesn't move
    await rezero_and_sync_steers(ctrls, steer_angle_est)

    net = NetworkInput()

    cur_vx = cur_vy = cur_omega = 0.0
    last_t = time.time()
    tick = 0

    faulted = set()

    try:
        while True:
            tick += 1
            now = time.time()
            dt = max(now - last_t, 0.001)
            last_t = now

            net.update()

            # If no packets ever received, keep everything fully stopped
            if net.packet_count == 0:
                await safe_stop_all(ctrls)
                await asyncio.sleep(CONTROL_DT)
                continue

            # Apply deadzone + clamp
            target_vx = clamp(apply_deadzone(net.vx), -1.0, 1.0) * MAX_V
            target_vy = clamp(apply_deadzone(net.vy), -1.0, 1.0) * MAX_V
            target_omega = clamp(apply_deadzone(net.omega), -1.0, 1.0) * MAX_OMEGA

            # Ramp
            cur_vx = ramp(cur_vx, target_vx, ACCEL_RATE, dt)
            cur_vy = ramp(cur_vy, target_vy, ACCEL_RATE, dt)
            cur_omega = ramp(cur_omega, target_omega, ROT_ACCEL_RATE, dt)

            if DEBUG_COMMAND and tick % 10 == 0:
                print(f"[CMD] vx={cur_vx:+.2f} vy={cur_vy:+.2f} omega={cur_omega:+.2f}")

            # If basically idle, STOP drives and HOLD steers where they are (no re-aiming to 0)
            cmd_mag = max(abs(cur_vx), abs(cur_vy), abs(cur_omega))
            if cmd_mag < IDLE_CMD_EPS:
                tasks = []
                # drives off
                for s in SWERVE_IDS:
                    tasks.append(send_stop_noreply(ctrls[f"{s}_drive"]))
                # steers hold last estimate
                for s in SWERVE_IDS:
                    ste = ctrls[f"{s}_steer"]
                    cmd_steer_rot = wheel_rad_to_motor_rot(s, steer_angle_est[s])
                    tasks.append(send_position_noreply(
                        ste,
                        position=cmd_steer_rot,
                        velocity_limit=STEER_VEL_LIMIT,
                        maximum_torque=STEER_MAX_TORQUE,
                        watchdog_timeout=0.2,
                    ))
                await asyncio.gather(*tasks, return_exceptions=True)
                await asyncio.sleep(CONTROL_DT)
                continue

            speeds, angles = calculate_swerve(cur_vx, cur_vy, cur_omega)

            tasks = []
            for side in SWERVE_IDS:
                drv = ctrls[f"{side}_drive"]
                ste = ctrls[f"{side}_steer"]
                drv_key = f"{side}_drive"
                ste_key = f"{side}_steer"

                spd = speeds[side]
                ang = angles[side]

                new_angle, opt_spd = optimize_swerve(ang, spd, steer_angle_est[side])
                steer_angle_est[side] = new_angle
                
                cmd_steer_rot = wheel_rad_to_motor_rot(side, new_angle)
                cmd_drive_vel = DRIVE_SIGN[side] * opt_spd  # NOTE: this is "normalized". scale if needed.

                if DEBUG_MODULES and tick % 10 == 0:
                    print(f"[{side}] spd={spd:+.2f} ang={ang:+.2f} steer_rot={cmd_steer_rot:+.3f} drv_vel={cmd_drive_vel:+.2f}")

                # Drive command
                if drv_key in faulted:
                    tasks.append(send_stop_noreply(drv, clear_faults=True))
                else:
                    tasks.append(send_position_noreply(
                        drv,
                        position=math.nan,
                        velocity=cmd_drive_vel,
                        maximum_torque=DRIVE_MAX_TORQUE,
                        accel_limit=ACCEL_RATE,
                        watchdog_timeout=0.2,
                    ))

                # Steer command
                if ste_key in faulted:
                    tasks.append(send_stop_noreply(ste, clear_faults=True))
                else:
                    tasks.append(send_position_noreply(
                        ste,
                        position=cmd_steer_rot,
                        velocity_limit=STEER_VEL_LIMIT,
                        maximum_torque=STEER_MAX_TORQUE,
                        watchdog_timeout=0.2,
                    ))

            await asyncio.gather(*tasks, return_exceptions=True)

            # Telemetry (5 Hz): update steer estimate from measured position + faults
            if tick % TELEMETRY_EVERY_N_TICKS == 0:
                names = list(ctrls.keys())
                states = await asyncio.gather(*[ctrls[n].query() for n in names], return_exceptions=True)
                st_by_name = {n: st for n, st in zip(names, states)}

                for name, st in st_by_name.items():
                    if isinstance(st, Exception) or st is None:
                        continue
                    mode = st.values.get(moteus.Register.MODE)
                    if mode == moteus.Mode.FAULT:
                        fault = st.values.get(moteus.Register.FAULT, "Unknown")
                        if name not in faulted:
                            print(f"[FAULT] {name}: {fault}")
                        faulted.add(name)
                    else:
                        faulted.discard(name)

                for side in SWERVE_IDS:
                    st = st_by_name.get(f"{side}_steer")
                    if isinstance(st, Exception) or st is None:
                        continue
                    pos_rot = st.values.get(moteus.Register.POSITION, None)
                    if pos_rot is None:
                        continue
                    steer_angle_est[side] = motor_rot_to_wheel_rad(side, float(pos_rot))

            await asyncio.sleep(CONTROL_DT)

    finally:
        print("\nStopping all motors...")
        await safe_stop_all(ctrls)
        print("Done.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_CAN_PORT,
                    help="fdcanusb serial device (e.g. /dev/ttyACM0 or /dev/serial/by-id/...)")
    args = ap.parse_args()

    try:
        asyncio.run(main(args.port))
    except KeyboardInterrupt:
