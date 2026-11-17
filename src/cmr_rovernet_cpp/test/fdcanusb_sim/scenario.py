#!/usr/bin/env python3
"""Scripted operator session (one input change per step, inputs republished at
20 Hz like the real controller); writes step markers '<monotonic> STEP <name>'
and logs /cmd_vel, /cmd_vel/teleop and /cmd_vel/estop.
usage: scenario.py <marker log> <mux pid file>"""
import os, signal, sys, time
import rclpy
from rclpy.node import Node
from cmr_msgs.msg import ControllerReading, DriveCommand
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, String

markers = open(sys.argv[1], "w", buffering=1)
mux_pid_file = sys.argv[2]
rclpy.init()
n = Node("scenario")
axes_pub = n.create_publisher(TwistStamped, "/controller/drives/axes", 10)
btn_pub = n.create_publisher(ControllerReading, "/controller/drives/buttons", 10)
src_pub = n.create_publisher(String, "/cmd_vel/source", 10)
auto_pub = n.create_publisher(DriveCommand, "/cmd_vel/autonomy", 10)
log = open(sys.argv[1] + ".topics", "w", buffering=1)
n.create_subscription(DriveCommand, "/cmd_vel/teleop", lambda m: log.write(
    f"{time.monotonic():.4f} teleop {m.vx!r} {m.vy!r} {m.omega!r} {m.speed_rps!r}\n"), 50)
n.create_subscription(Bool, "/cmd_vel/estop", lambda m: log.write(
    f"{time.monotonic():.4f} estop {m.data}\n"), 50)
n.create_subscription(DriveCommand, "/cmd_vel", lambda m: log.write(
    f"{time.monotonic():.4f} cmd_vel {m.vx!r} {m.vy!r} {m.omega!r} {m.speed_rps!r}\n"), 50)

state = dict(axes=(0.0, 0.0, 0.0), btn=[0] * 8, dpad=8, publish=True, auto=None)


def hold(name, seconds=0.8, **changes):
    state.update(changes)
    markers.write(f"{time.monotonic():.4f} STEP {name}\n")
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if state["publish"]:
            b = ControllerReading()
            b.button_array = list(state["btn"])
            b.dpad = state["dpad"]
            btn_pub.publish(b)
            a = TwistStamped()
            a.twist.linear.x, a.twist.linear.y, a.twist.angular.x = state["axes"]
            axes_pub.publish(a)
        if state["auto"] is not None:
            vx, vy, om, rps = state["auto"]
            auto_pub.publish(DriveCommand(vx=vx, vy=vy, omega=om, speed_rps=rps))
        rclpy.spin_once(n, timeout_sec=0.0)
        t = time.monotonic() + 0.05
        while time.monotonic() < t:
            rclpy.spin_once(n, timeout_sec=0.005)


def btn(l1=0, r1=0, l2=0, r2=0, tri=0):
    b = [0] * 8
    b[0], b[1], b[2], b[3], b[7] = l1, r1, l2, r2, tri
    return b


FWD = (0.0, -2.5, 0.0)          # linear.y=-2.5 -> vx=+1
REV = (0.0, 2.5, 0.0)
MIX = (1.25, -2.0, 1.0)         # vy=0.5 vx=0.8 omega=0.4

hold("startup_idle", 3.0, publish=False)
hold("neutral_inputs", publish=True)
hold("r2_speed_no_stick", btn=btn(r2=255))
hold("forward", axes=FWD)
hold("mix", axes=MIX)
hold("dpad_right_ackermann", dpad=2)
hold("dpad_neutral_stays_ackermann", dpad=8)
hold("dpad_diagonal_ignored", dpad=3)
hold("dpad_down_point_turn", dpad=4)
hold("dpad_left_steady_heading", dpad=6)
hold("dpad_up_translation_rotation", dpad=0)
hold("dpad_neutral", dpad=8)
hold("l2_half_speed", btn=btn(l2=200))
hold("r1_reverse_speed", btn=btn(r1=1))
hold("l1_triple_speed", btn=btn(l1=1))
hold("reverse_stick_flips_speed_sign", axes=REV)
hold("stick_released_keeps_sign", axes=(0.0, 0.0, 0.0))
hold("r2_speed_sign_remembered", btn=btn(r2=255))
hold("mix_again", axes=MIX)
hold("ESTOP_l1_triangle", btn=btn(l1=1, tri=1))
hold("estop_held_axes_change", axes=FWD)
hold("estop_dpad_ignored", dpad=2)
hold("estop_dpad_neutral", dpad=8)
hold("estop_only_triangle_released", btn=btn(l1=1))
hold("estop_released", btn=btn())
hold("drive_after_estop", btn=btn(r2=255))
hold("point_turn_after_estop", dpad=4)
hold("back_to_translation", dpad=0)
hold("dpad_neutral2", dpad=8)
hold("mix3", axes=MIX)
hold("CONTROLLER_SILENT_mux_timeout", 1.5, publish=False)
hold("controller_back", publish=True)
hold("select_autonomy_no_cmd", 1.0)
src_pub.publish(String(data="autonomy"))
hold("autonomy_selected_teleop_ignored", 1.0)
hold("autonomy_cmd", auto=(0.5, 0.25, -0.3, 2.0))
hold("autonomy_cmd_overspeed_clamped", auto=(2.0, -3.0, 0.3, 50.0))
hold("autonomy_point_turn_mode", dpad=4)
hold("autonomy_neutral", dpad=0)
hold("autonomy_silent_timeout", 1.2, auto=None)
src_pub.publish(String(data="tank"))
hold("bad_source_ignored")
src_pub.publish(String(data="teleop"))
hold("teleop_again", dpad=8)
os.kill(int(open(mux_pid_file).read()), signal.SIGKILL)
hold("MUX_KILLED_command_timeout", 1.5)
markers.write(f"{time.monotonic():.4f} STEP end\n")
