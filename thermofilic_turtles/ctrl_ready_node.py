#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

from thermofilic_turtle.srv import GetTempByCoord


class ControllersReady(Node):
    def __init__(self):
        super().__init__("ctrl_ready")

        self.declare_parameter(name="turtles_count", value=10)
        self.turtles_count = self.get_parameter("turtles_count").value

        self.get_logger().info(
            f"Controllers Ready Node has been started - waiting for {self.turtles_count} turtles"
        )
        self.ctrls_ready = 0
        self.create_subscription(Empty, "ctrl_ready", self.check_ready_callback, 10)

    def check_ready_callback(self, msg):
        """Callback function for checking if all turtles are ready."""
        self.ctrls_ready += 1
        self.get_logger().info(
            f"Ready controllers: {self.ctrls_ready}/{self.turtles_count}"
        )
        if self.ctrls_ready == self.turtles_count:
            self.get_logger().info("All turtle's controllers are ready")
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ControllersReady()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
