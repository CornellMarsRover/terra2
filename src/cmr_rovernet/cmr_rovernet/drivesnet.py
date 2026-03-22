import math
import asyncio
import threading
import concurrent.futures

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import ControllerReading

import moteus

from cmr_rovernet.rovernet_utils import (
    parse_toml,
    scale_value,
    ROVER_LENGTH,
    ROVER_WIDTH,
    L1,
    L2,
    L2_MIN,
    TRIANGLE,
)


class DrivesNet(Node):
    """
    Minimal swerve drive node using explicit moteus transport on a dedicated asyncio loop.

    Drive motors:
      1 = FL
      2 = BL
      3 = FR
      4 = BR

    Swerve motors:
      5 = FL
      6 = BL
      7 = FR
      8 = BR
    """

    def __init__(self):
        super().__init__("drivesnet")

        self.create_subscription(
            TwistStamped,
            "/drives_controller/cmd_vel",
            self.cmd_vel_callback,
            10,
        )
        self.create_subscription(
            ControllerReading,
            "/drives_controller/cmd_buttons",
            self.buttons_callback,
            10,
        )

        self.logger = self.get_logger()

        drives_net_table = parse_toml("drivesnet")
        node_cfg = drives_net_table["node"]

        self.MOTOR_MAX_SPEED = float(node_cfg["motor_max_speed"])
        self.MAX_ACCELERATION = float(node_cfg["acceleration_limit"])
        self.MAX_TORQUE = float(node_cfg["torque_limit"])

        self.velocity = 0.0

        self._last_send_future = None
        self._moteus_ready = threading.Event()
        self._shutdown = False

        self._start_async_thread()
        self._moteus_ready.wait(timeout=5.0)

        if not self._moteus_ready.is_set():
            raise RuntimeError("Timed out initializing moteus transport/controllers")

        self.logger.info("drivesnet started with explicit Fdcanusb transport")

    def _start_async_thread(self):
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._async_thread_main,
            daemon=True,
        )
        self._thread.start()

    def _async_thread_main(self):
        asyncio.set_event_loop(self._loop)
        self._loop.create_task(self._async_init())
        self._loop.run_forever()

    async def _async_init(self):
        try:
            self.transport = moteus.Fdcanusb(path="/dev/ttyACM0")

            qr = moteus.QueryResolution()
            qr.mode = moteus.INT8
            qr.position = moteus.F32
            qr.velocity = moteus.F32
            qr.torque = moteus.F32
            qr.q_current = moteus.F32

            self.drive_fl = moteus.Controller(id=1, transport=self.transport, query_resolution=qr)
            self.drive_bl = moteus.Controller(id=2, transport=self.transport, query_resolution=qr)
            self.drive_fr = moteus.Controller(id=3, transport=self.transport, query_resolution=qr)
            self.drive_br = moteus.Controller(id=4, transport=self.transport, query_resolution=qr)

            self.swerve_fl = moteus.Controller(id=5, transport=self.transport, query_resolution=qr)
            self.swerve_bl = moteus.Controller(id=6, transport=self.transport, query_resolution=qr)
            self.swerve_fr = moteus.Controller(id=7, transport=self.transport, query_resolution=qr)
            self.swerve_br = moteus.Controller(id=8, transport=self.transport, query_resolution=qr)

            self._moteus_ready.set()
        except Exception as e:
            self.logger.error(f"moteus async init failed: {e}")
            self._moteus_ready.set()
            raise

    def deadband(self, value: float, threshold: float = 0.1) -> float:
        return 0.0 if abs(value) < threshold else value

    def angle_deg_to_moteus_pos(self, angle_deg: float) -> float:
        # Preserve your current scaling behavior
        return (angle_deg / 360.0) * 50.0

    def wheel_angles_and_speeds(self, vx: float, vy: float, omega: float, L: float, W: float):
        R = math.sqrt(L**2 + W**2)

        A = vy - omega * (L / R)
        B = vy + omega * (L / R)
        C = vx - omega * (W / R)
        D = vx + omega * (W / R)

        ws1 = math.sqrt(B**2 + C**2)
        ws2 = math.sqrt(B**2 + D**2)
        ws3 = math.sqrt(A**2 + D**2)
        ws4 = math.sqrt(A**2 + C**2)

        wa1_deg = math.degrees(math.atan2(B, C))
        wa2_deg = math.degrees(math.atan2(B, D))
        wa3_deg = math.degrees(math.atan2(A, D))
        wa4_deg = math.degrees(math.atan2(A, C))

        max_ws = max(ws1, ws2, ws3, ws4)
        if max_ws > 1.0:
            ws1 /= max_ws
            ws2 /= max_ws
            ws3 /= max_ws
            ws4 /= max_ws

        wa1 = self.angle_deg_to_moteus_pos(wa1_deg)
        wa2 = self.angle_deg_to_moteus_pos(wa2_deg)
        wa3 = self.angle_deg_to_moteus_pos(wa3_deg)
        wa4 = self.angle_deg_to_moteus_pos(wa4_deg)

        return ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4

    async def send_drive_async(self, controller, motor_id: int, velocity: float, name: str):
        try:
            await controller.set_position(
                position=math.nan,
                velocity=velocity,
                maximum_torque=self.MAX_TORQUE,
                accel_limit=self.MAX_ACCELERATION,
                velocity_limit=self.MOTOR_MAX_SPEED,
                query=True,
            )
        except Exception as e:
            self.logger.error(f"drive send failed: {name} id={motor_id} vel={velocity} err={e}")

    async def send_swerve_async(self, controller, motor_id: int, position: float, name: str):
        try:
            await controller.set_position(
                position=position,
                maximum_torque=10.0,
                accel_limit=40.0,
                velocity_limit=60.0,
                query=True,
            )
        except Exception as e:
            self.logger.error(f"swerve send failed: {name} id={motor_id} pos={position} err={e}")

    async def stop_drive_async(self, controller, motor_id: int, name: str):
        try:
            await controller.set_stop()
        except Exception as e:
            self.logger.error(f"stop failed: {name} id={motor_id} err={e}")

    async def send_all_async(self, ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4):
        await self.send_drive_async(self.drive_fl, 1, -self.velocity * ws2, "FL drive")
        await self.send_drive_async(self.drive_bl, 2, -self.velocity * ws3, "BL drive")
        await self.send_drive_async(self.drive_fr, 3,  self.velocity * ws4, "FR drive")
        await self.send_drive_async(self.drive_br, 4,  self.velocity * ws1, "BR drive")

        await self.send_swerve_async(self.swerve_bl, 6, wa3, "BL swerve")
        await self.send_swerve_async(self.swerve_fr, 7, wa1, "FR swerve")
        await self.send_swerve_async(self.swerve_br, 8, wa4, "BR swerve")
        await self.send_swerve_async(self.swerve_fl, 5, wa2, "FL swerve")

    async def stop_all_drive_async(self):
        await self.stop_drive_async(self.drive_fl, 1, "FL drive")
        await self.stop_drive_async(self.drive_bl, 2, "BL drive")
        await self.stop_drive_async(self.drive_fr, 3, "FR drive")
        await self.stop_drive_async(self.drive_br, 4, "BR drive")

    def submit_async(self, coro):
        if self._shutdown:
            return None
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    def cmd_vel_callback(self, msg: TwistStamped):
        if not self._moteus_ready.is_set():
            return

        ly = self.deadband(msg.twist.linear.y)
        lx = self.deadband(msg.twist.linear.x)
        rx = self.deadband(msg.twist.angular.x)

        ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4 = self.wheel_angles_and_speeds(
            -ly,
            lx,
            rx,
            ROVER_LENGTH,
            ROVER_WIDTH,
        )

        self.logger.info(
            f"vel={self.velocity:.3f} "
            f"FR_drive={self.velocity * ws4:.3f} "
            f"BL_swerve={wa3:.3f} "
            f"FR_swerve={wa1:.3f}"
        )

        # Optional: skip queuing if previous send still running
        if self._last_send_future is not None and not self._last_send_future.done():
            return

        self._last_send_future = self.submit_async(
            self.send_all_async(ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4)
        )

    def buttons_callback(self, msg: ControllerReading):
        """
        Expected layout:
          [L1, R1, L2, R2, square, cross, circle, triangle]
        """
        if len(msg.button_array) < 8:
            self.logger.warn("button_array too short")
            return

        l1 = msg.button_array[0]
        l2 = msg.button_array[2]
        r2 = msg.button_array[3]
        triangle = msg.button_array[7]

        if 0 <= r2 <= 255:
            self.velocity = scale_value(float(r2), 0.0, 255.0, 0.0, self.MOTOR_MAX_SPEED)

        if L2_MIN <= l2 <= L2:
            self.velocity = -scale_value(float(l2), float(L2_MIN), float(L2), 0.0, self.MOTOR_MAX_SPEED)

        if l1 == L1 and triangle == TRIANGLE:
            self.velocity = 0.0
            fut = self.submit_async(self.stop_all_drive_async())
            self.logger.info("Drive stop requested")
            if fut is not None:
                try:
                    fut.result(timeout=1.0)
                except concurrent.futures.TimeoutError:
                    self.logger.warn("Timed out waiting for stop commands")
                except Exception as e:
                    self.logger.error(f"stop batch failed: {e}")

    def destroy_node(self):
        self._shutdown = True

        try:
            fut = self.submit_async(self.stop_all_drive_async())
            if fut is not None:
                try:
                    fut.result(timeout=1.0)
                except Exception:
                    pass
        except Exception:
            pass

        try:
            if hasattr(self, "_loop"):
                self._loop.call_soon_threadsafe(self._loop.stop)
        except Exception:
            pass

        try:
            if hasattr(self, "_thread") and self._thread.is_alive():
                self._thread.join(timeout=1.0)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DrivesNet()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()