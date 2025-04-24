#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class ControllersReady(Node):
    def __init__(self):
        super().__init__("ctrl_ready")

        self.declare_parameter(name="turtles_count", value=10)
        self.turtles_count = self.get_parameter("turtles_count").value

        self.get_logger().info(
            f"Controllers Ready Node has been started - waiting for {self.turtles_count} turtles"
        )
        self.ctrls_ready = 0
        qos_profile = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            Empty, "ctrl_ready", self.check_ready_callback, qos_profile
        )

    def check_ready_callback(self, msg):
        """Callback function for checking if all turtles are ready."""
        self.ctrls_ready += 1
        self.get_logger().info(
            f"Ready controllers: {self.ctrls_ready}/{self.turtles_count}"
        )
        if self.ctrls_ready == self.turtles_count:
            self.get_logger().info("All turtle's controllers are ready")
            # End spinning
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)

    node = ControllersReady()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        node.get_logger().info("Quitting Controllers Ready Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
