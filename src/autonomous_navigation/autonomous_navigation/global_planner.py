#!/usr/bin/env python3
"""Global planner node: the consumer /autonomy/target/global never had.

The state machine already publishes every mission target twice, once on
/autonomy/target/local and once on /autonomy/target/global, and until now
nothing subscribed to the global one. This node fills that slot. It keeps a
persistent map of every costmap it has been sent, routes over it, and republishes
a reachable sub-goal on /autonomy/target/local, so the local planner keeps
owning obstacle avoidance inside its own window and stops being handed a goal
that sits behind a wall.

Run it instead of, not alongside, the state machine's own local publication:
launch the state machine with `publish_local_target:=false`, which is what
`sim_autonomy_global.launch.py` does.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import TwistStamped

from autonomous_navigation.planner_core import parse_costmap, parse_planar_target
from autonomous_navigation.global_planner_core import MapConfig, WorldMap, plan


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')

        self.declare_parameter('cell_size', 0.5)
        self.declare_parameter('occupied_threshold', 4.0)
        self.declare_parameter('blocked_cost', 4.0)
        self.declare_parameter('evidence_hit', 3.0)
        self.declare_parameter('evidence_miss', 1.0)
        self.declare_parameter('evidence_ceiling', 12.0)
        self.declare_parameter('lookahead', 6.0)
        self.declare_parameter('inflation_radius', 1)
        self.declare_parameter('search_margin', 4.0)
        self.declare_parameter('arrival_radius', 1.0)
        self.declare_parameter('plan_period', 1.0)

        def value(name):
            return self.get_parameter(name).value

        self.world = WorldMap(MapConfig(
            cell_size=float(value('cell_size')),
            occupied_threshold=float(value('occupied_threshold')),
            hit=float(value('evidence_hit')),
            miss=float(value('evidence_miss')),
            ceiling=float(value('evidence_ceiling')),
            blocked_cost=float(value('blocked_cost')),
        ))
        self.lookahead = float(value('lookahead'))
        self.inflation_radius = int(value('inflation_radius'))
        self.search_margin = float(value('search_margin'))
        self.arrival_radius = float(value('arrival_radius'))

        self.position = (0.0, 0.0)
        self.target = None
        self.last_reason = None

        self.create_subscription(
            Float32MultiArray, '/autonomy/target/global', self.target_callback, 10)
        self.create_subscription(
            Float32MultiArray, '/autonomy/costmap', self.costmap_callback, 10)
        self.create_subscription(
            TwistStamped, '/autonomy/pose/robot/global', self.pose_callback, 10)

        self.subgoal_pub = self.create_publisher(
            Float32MultiArray, '/autonomy/target/local', 10)
        self.route_pub = self.create_publisher(
            Float32MultiArray, '/autonomy/path/global', 10)
        self.reason_pub = self.create_publisher(
            String, '/autonomy/path/global_reason', 10)

        self.create_timer(float(value('plan_period')), self.replan)
        self.get_logger().info('Global planner node initialized')

    # -- callbacks --------------------------------------------------------
    def target_callback(self, msg):
        """Latch the mission waypoint the state machine is currently chasing."""
        try:
            target, _ = parse_planar_target(msg.data)
        except ValueError as error:
            self.get_logger().error(f'Ignoring malformed global target: {error}')
            return
        if target != self.target:
            self.target = target
            self.get_logger().info(f'New global target: {target}')

    def costmap_callback(self, msg):
        """Fold one local costmap frame into the persistent map."""
        try:
            costs, _ = parse_costmap(msg.data, self.world.config.blocked_cost)
        except ValueError as error:
            self.get_logger().error(f'Ignoring malformed costmap: {error}')
            return
        self.world.integrate(costs)

    def pose_callback(self, msg):
        """Track the rover and record the ground it has proved is drivable."""
        self.position = (msg.twist.linear.x, msg.twist.linear.y)
        self.world.mark_visited(self.position)

    # -- planning ---------------------------------------------------------
    def replan(self):
        """Publish the sub-goal the local planner should drive toward."""
        if self.target is None:
            return
        result = plan(
            self.position,
            self.target,
            self.world,
            lookahead=self.lookahead,
            inflation_radius=self.inflation_radius,
            margin=self.search_margin,
            arrival=self.arrival_radius,
        )
        self.subgoal_pub.publish(Float32MultiArray(
            data=[float(result.subgoal[0]), float(result.subgoal[1])]))
        self.route_pub.publish(Float32MultiArray(
            data=[float(v) for point in result.route for v in point]))
        self.reason_pub.publish(String(data=result.reason))
        if result.reason != self.last_reason:
            self.get_logger().info(
                f'Global plan {result.reason}: sub-goal {result.subgoal}, '
                f'{len(result.route)} cells, target {self.target}')
            self.last_reason = result.reason


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
