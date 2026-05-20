import sys

import rclpy
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from rclpy.node import Node


class MoveGroupProbe(Node):
    def __init__(self) -> None:
        super().__init__("move_group_probe")
        self.client = ActionClient(self, MoveGroup, "/move_action")


def main() -> None:
    rclpy.init()
    node = MoveGroupProbe()
    try:
        node.get_logger().info("waiting for MoveIt /move_action action server")
        if node.client.wait_for_server(timeout_sec=20.0):
            node.get_logger().info("MoveIt /move_action is available")
            raise SystemExit(0)
        node.get_logger().error("MoveIt /move_action was not available within 20 seconds")
        raise SystemExit(1)
    except KeyboardInterrupt:
        raise SystemExit(130)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except SystemExit as exc:
        raise exc
    except Exception as exc:
        print(f"probe failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
