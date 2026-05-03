#!/usr/bin/env python3

from pathlib import Path
from typing import List

import rclpy
from gazebo_msgs.srv import SpawnEntity
from rclpy.node import Node


class DemoEntitySpawner(Node):
    def __init__(self) -> None:
        super().__init__("spawn_demo_entities")

        self.declare_parameter("robot_entity", "drives")
        self.declare_parameter("robot_file", "")
        self.declare_parameter("robot_x", 0.0)
        self.declare_parameter("robot_y", 0.0)
        self.declare_parameter("robot_z", 0.05)
        self.declare_parameter("obstacle_file", "")
        self.declare_parameter("obstacle_entities", [""])
        self.declare_parameter("obstacle_xs", [0.0])
        self.declare_parameter("obstacle_ys", [0.0])
        self.declare_parameter("obstacle_zs", [0.5])

        self.client = self.create_client(SpawnEntity, "/spawn_entity")
        self.get_logger().info("Waiting for /spawn_entity service...")
        self.client.wait_for_service()

    def spawn_all(self) -> int:
        robot_file = Path(str(self.get_parameter("robot_file").value))
        obstacle_file = Path(str(self.get_parameter("obstacle_file").value))

        self._spawn_from_file(
            entity=str(self.get_parameter("robot_entity").value),
            file_path=robot_file,
            x=float(self.get_parameter("robot_x").value),
            y=float(self.get_parameter("robot_y").value),
            z=float(self.get_parameter("robot_z").value),
        )

        obstacle_entities: List[str] = list(self.get_parameter("obstacle_entities").value)
        obstacle_xs: List[float] = [float(value) for value in self.get_parameter("obstacle_xs").value]
        obstacle_ys: List[float] = [float(value) for value in self.get_parameter("obstacle_ys").value]
        obstacle_zs: List[float] = [float(value) for value in self.get_parameter("obstacle_zs").value]

        for entity, x, y, z in zip(obstacle_entities, obstacle_xs, obstacle_ys, obstacle_zs):
            if not entity:
                continue
            self._spawn_from_file(entity=entity, file_path=obstacle_file, x=x, y=y, z=z)

        return 0

    def _spawn_from_file(self, *, entity: str, file_path: Path, x: float, y: float, z: float) -> None:
        xml = file_path.read_text()
        request = SpawnEntity.Request()
        request.name = entity
        request.xml = xml
        request.robot_namespace = entity
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z

        self.get_logger().info(
            f"Spawning {entity} from {file_path} at ({x:.2f}, {y:.2f}, {z:.2f})"
        )
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if not future.result():
            raise RuntimeError(f"Spawn service for {entity} returned no result")
        response = future.result()
        if not response.success:
            raise RuntimeError(f"Failed to spawn {entity}: {response.status_message}")
        self.get_logger().info(f"Spawned {entity}: {response.status_message}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoEntitySpawner()
    exit_code = 0
    try:
        exit_code = node.spawn_all()
    except Exception as exc:  # pragma: no cover - launch-surface diagnostics
        node.get_logger().error(f"Demo entity spawn failed: {exc!r}")
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
