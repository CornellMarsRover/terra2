import asyncio
import numpy as np
import moteus
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# ==========================================
# PHYSICAL SPECS
# ==========================================
L_BASE = 0.1016
L1 = 15.25 * 0.0254
L2 = 13.5 * 0.0254
L_GRIP = 7 * 0.0254
OFFSET = 3.5 * 0.0254

LIMITS = [
    (-170, 170),
    (-120, 10),
    (-150, 150),
    (-180, 180),
    (-100, 100),
    (-180, 180)
]

# ============================================================
# Math Helpers
# ============================================================
def Rx(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]])

def Ry(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]])

def Rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])

def rot2axisangle(R):
    trace = np.trace(R)
    if trace > 3.0: trace = 3.0
    if trace < -1.0: trace = -1.0
    angle = np.arccos((trace - 1) / 2)
    if angle < 1e-6: return np.zeros(3)
    axis = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]) / (2*np.sin(angle))
    return axis * angle

# ============================================================
# CORE KINEMATICS
# ============================================================
class RobotArm:
    def __init__(self):
        self.q = np.zeros(6)

    def forward_kinematics_chain(self, q):
        transforms = []
        positions = []
        z_axes = []

        T = np.eye(4)
        transforms.append(T.copy()); positions.append(T[:3, 3]); z_axes.append(T[:3, 2])

        T = T @ Rz(q[0])
        T_shoulder_static = np.eye(4)
        T_shoulder_static[:3, 3] = [0, OFFSET, L_BASE]
        T = T @ T_shoulder_static
        transforms.append(T.copy()); positions.append(T[:3, 3]); z_axes.append(T[:3, 1])

        T = T @ Ry(q[1])
        T_elbow_static = np.eye(4)
        T_elbow_static[:3, 3] = [L1, 0, 0]
        T = T @ T_elbow_static
        transforms.append(T.copy()); positions.append(T[:3, 3]); z_axes.append(T[:3, 1])

        T = T @ Ry(q[2])
        T_forearm_static = np.eye(4)
        T_forearm_static[:3, 3] = [L2, 0, 0]
        T = T @ T_forearm_static
        transforms.append(T.copy()); positions.append(T[:3, 3]); z_axes.append(T[:3, 0])

        T = T @ Rx(q[3])
        transforms.append(T.copy()); positions.append(T[:3, 3]); z_axes.append(T[:3, 1])

        T = T @ Ry(q[4])
        transforms.append(T.copy()); positions.append(T[:3, 3]); z_axes.append(T[:3, 0])

        T = T @ Rx(q[5])
        T_tool_static = np.eye(4)
        T_tool_static[:3, 3] = [L_GRIP, 0, 0]
        T = T @ T_tool_static
        transforms.append(T.copy()); positions.append(T[:3, 3])

        return transforms, positions, z_axes

    def get_jacobian(self, q):
        _, positions, z_axes = self.forward_kinematics_chain(q)
        p_ee = positions[-1]
        J = np.zeros((6, 6))
        for i in range(6):
            z, p = z_axes[i], positions[i]
            J[:3, i] = np.cross(z, p_ee - p)
            J[3:, i] = z
        return J

    def inverse_kinematics_numerical(self, target_pos, target_rpy, q_seed=None):
        R_target_4x4 = Rz(np.radians(target_rpy[2])) @ Ry(np.radians(target_rpy[1])) @ Rx(np.radians(target_rpy[0]))
        R_target = R_target_4x4[:3, :3]

        # FIX #4 (warm-start): use provided seed, fall back to neutral pose
        if q_seed is not None:
            q_curr = q_seed.copy()
        else:
            q_curr = np.array([0.0, np.radians(10), np.radians(10), 0.0, 0.0, 0.0])

        best_q = q_curr.copy()
        best_err = float('inf')
        best_pos = np.zeros(3)

        for _ in range(150):
            transforms, positions, _ = self.forward_kinematics_chain(q_curr)
            p_current, R_current = positions[-1], transforms[-1][:3, :3]

            err_pos = target_pos - p_current
            err_rot = rot2axisangle(R_target @ R_current.T)
            total_err = np.linalg.norm(err_pos) + 0.1 * np.linalg.norm(err_rot)

            if total_err < best_err:
                best_err, best_q, best_pos = total_err, q_curr.copy(), p_current

            if np.linalg.norm(err_pos) < 0.005 and np.linalg.norm(err_rot) < 0.05:
                break

            J = self.get_jacobian(q_curr)
            delta_q = np.clip(np.linalg.pinv(J, rcond=1e-2) @ np.hstack((err_pos, err_rot)), -0.3, 0.3)
            q_curr += delta_q * 0.5

            for i in range(6):
                q_curr[i] = np.clip(q_curr[i], np.radians(LIMITS[i][0]), np.radians(LIMITS[i][1]))

        self.q = best_q
        return self.q, best_pos

# ============================================================
# MOTEUS HELPERS
# ============================================================
def to_moteus_units(angles_deg):
    out = []
    for i, deg in enumerate(angles_deg):
        scale = 100 if i < 3 else 50
        out.append((deg / 360.0) * scale)
    return out


# FIX #2: Controllers are created once and reused for the lifetime of the node.
# FIX #8: stop_all_motors() sends set_stop() to every controller so they de-energize
#         cleanly on exit or fault.
class MoteusInterface:
    """Owns the moteus transport and all six controllers. Created once."""

    def __init__(self):
        # A single shared Router/transport is far more efficient than one per controller.
        self.transport = moteus.Fdcanusb()

        self.controllers = {
            'base':           moteus.Controller(id=9,  transport=self.transport),
            'shoulder':       moteus.Controller(id=10, transport=self.transport),
            'elbow':          moteus.Controller(id=11, transport=self.transport),
            'wrist_rotate_1': moteus.Controller(id=12, transport=self.transport),
            'wrist_tilt':     moteus.Controller(id=13, transport=self.transport),
            'wrist_rotate_2': moteus.Controller(id=14, transport=self.transport),
        }

    async def move_joints(self, positions):
        """Send position commands to all six joints concurrently."""
        c = self.controllers
        await asyncio.gather(
            c['base'].set_position(
                position=positions[0], velocity=math.nan,
                maximum_torque=0.7, velocity_limit=5.0, accel_limit=5.0,
                feedforward_torque=0.0),
            c['shoulder'].set_position(
                position=positions[1], velocity=math.nan,
                maximum_torque=0.7, velocity_limit=10.0, accel_limit=10.0,
                feedforward_torque=0.0),
            c['elbow'].set_position(
                position=positions[2], velocity=math.nan,
                maximum_torque=0.7, velocity_limit=10.0, accel_limit=10.0,
                feedforward_torque=0.0),
            c['wrist_rotate_1'].set_position(
                position=positions[3], velocity=math.nan,
                maximum_torque=0.7, velocity_limit=5.0, accel_limit=5.0,
                feedforward_torque=0.0),
            c['wrist_tilt'].set_position(
                position=positions[4], velocity=math.nan,
                maximum_torque=0.7, velocity_limit=5.0, accel_limit=5.0,
                feedforward_torque=0.0),
            c['wrist_rotate_2'].set_position(
                position=positions[5], velocity=math.nan,
                maximum_torque=0.7, velocity_limit=5.0, accel_limit=5.0,
                feedforward_torque=0.0),
        )

    async def stop_all_motors(self):
        """FIX #8: De-energize every motor. Call on exit or fault."""
        await asyncio.gather(
            *[ctrl.set_stop() for ctrl in self.controllers.values()]
        )


# ============================================================
# ROS2 NODE
# ============================================================
class ArmFollower(Node):
    def __init__(self, asyncio_loop: asyncio.AbstractEventLoop):
        super().__init__('arm_follower')

        # FIX #1: store a reference to the main asyncio loop so we can schedule
        # coroutines from the ROS2 executor thread safely.
        self._loop = asyncio_loop

        # FIX #2: one MoteusInterface for the lifetime of this node.
        self.moteus_iface = MoteusInterface()

        # FIX #9: flag that prevents a new move from being dispatched while the
        # previous coroutine is still awaiting CAN responses.
        self._move_in_progress = False

        self.subscription = self.create_subscription(
            Pose, '/arm_target_pose', self.pose_callback, 10)

        self.latest_pose = None
        self.robot = RobotArm()
        self.current_q_rad = np.array([0.0, np.radians(10), np.radians(10), 0.0, 0.0, 0.0])
        self.current_motors = [0.0] * 6

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Arm follower node started")

    def pose_callback(self, msg: Pose):
        self.latest_pose = msg

    def control_loop(self):
        if self.latest_pose is None:
            return

        # FIX #9: skip this cycle if the previous move command hasn't finished.
        if self._move_in_progress:
            self.get_logger().debug("Skipping cycle: previous move still in progress")
            return

        target_pos = np.array([
            self.latest_pose.position.x,
            self.latest_pose.position.y,
            self.latest_pose.position.z,
        ])

        q = self.latest_pose.orientation
        target_rpy = self.quaternion_to_rpy(q.x, q.y, q.z, q.w)

        # Run IK, warm-started from the last solved configuration
        q_sol, actual_pos = self.robot.inverse_kinematics_numerical(
            target_pos, target_rpy, q_seed=self.current_q_rad)
        math_deg = np.degrees(q_sol)

        motor_shoulder = math_deg[1] + 90.0
        k_pitch, k_roll = math_deg[4], math_deg[5]
        diff_m1 = k_roll + k_pitch
        diff_m2 = k_roll - k_pitch

        final_motors = [
            math_deg[0],
            motor_shoulder,
            math_deg[2],
            math_deg[3],
            diff_m1,
            diff_m2,
        ]

        err = np.linalg.norm(target_pos - actual_pos)
        if err > 0.05:
            self.get_logger().warn(f"High IK error ({err:.4f} m), move cancelled")
            return

        moteus_pos = to_moteus_units(final_motors)

        # FIX #1: use run_coroutine_threadsafe so the coroutine is scheduled on
        # the asyncio event loop that is running in the main thread, not the
        # background executor thread where this callback is executing.
        # FIX #9: wrap the coroutine so _move_in_progress is set/cleared atomically.
        self._move_in_progress = True
        future = asyncio.run_coroutine_threadsafe(
            self._send_and_clear(moteus_pos), self._loop)

        # Log any exception that surfaced from the coroutine (non-blocking).
        future.add_done_callback(self._on_move_done)

        self.current_q_rad = q_sol
        self.current_motors = moteus_pos

    async def _send_and_clear(self, moteus_pos):
        """Await the move, then release the in-progress flag."""
        try:
            await self.moteus_iface.move_joints(moteus_pos)
        finally:
            # Always release the lock, even if move_joints raises.
            self._move_in_progress = False

    def _on_move_done(self, future):
        """Called in the asyncio thread when the coroutine finishes."""
        exc = future.exception()
        if exc:
            self.get_logger().error(f"move_joints raised an exception: {exc}")
            self._move_in_progress = False  # ensure flag is cleared on error path

    def quaternion_to_rpy(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return np.array([roll, pitch, yaw])

    def shutdown(self):
        """FIX #8: cleanly stop all motors before tearing down."""
        self.get_logger().info("Stopping all motors...")
        future = asyncio.run_coroutine_threadsafe(
            self.moteus_iface.stop_all_motors(), self._loop)
        try:
            future.result(timeout=3.0)  # wait up to 3 s for the stop to complete
            self.get_logger().info("All motors stopped.")
        except Exception as e:
            self.get_logger().error(f"Error stopping motors: {e}")


# ============================================================
# ENTRY POINT
# ============================================================
def main(args=None):
    rclpy.init(args=args)

    # The asyncio event loop lives on the main thread.
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # Pass the loop into the node so it can schedule coroutines thread-safely.
    node = ArmFollower(asyncio_loop=loop)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # ROS2 spin runs in a background thread; the asyncio loop runs on main.
        # FIX #1: run_coroutine_threadsafe bridges the two safely.
        import threading
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        loop.run_forever()

    except KeyboardInterrupt:
        pass

    finally:
        # FIX #8: always stop motors, even on crash or Ctrl-C.
        node.shutdown()

        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

        loop.close()


if __name__ == '__main__':
    main()