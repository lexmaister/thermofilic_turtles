#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

from thermofilic_turtle.srv import GetTempByCoord


class KinesisController(Node):
    def __init__(self):
        super().__init__("kinesis_controller")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("turle_num", 1),
                ("linear_vel_min", 0.2),
                ("linear_vel_max", 3.0),
                ("angular_vel_min", 0.2),
                ("angular_vel_max", 3.14 / 2),
                ("timer_frequency", 10),  # Hz
            ],
        )

        self.turtle_num = self.get_parameter("turle_num").value
        self.linear_vel_min = self.get_parameter("linear_vel_min").value
        self.linear_vel_max = self.get_parameter("linear_vel_max").value
        self.angular_vel_min = self.get_parameter("angular_vel_min").value
        self.angular_vel_max = self.get_parameter("angular_vel_max").value
        self.timer_frequency = self.get_parameter("timer_frequency").value

        # Service client for temperature
        self.temp_client = self.create_client(GetTempByCoord, "get_temp_by_coord")
        while not self.temp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_temp_by_coord service...")

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist, f"turtle{self.turtle_num}/cmd_vel", 10
        )

        # Timer for movement and temperature requests
        self.movement_timer = self.create_timer(
            1.0 / self.timer_frequency,
            self.timer_callback,
        )

        self.moving = True  # if turtle isn't spawned yet
        self.x = 0.0
        self.y = 0.0
        self._waiting_for_temp = False
        self._temp_future = None

        # while not self.temp_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Waiting for get_temp_by_coord service...")

        self.create_subscription(
            Pose, f"turtle{self.turtle_num}/pose", self.pose_callback, 10
        )

        # Publish ready message
        self.ready_pub = self.create_publisher(Empty, "ctrl_ready", 10)
        self.ready_pub.publish(Empty())
        self.get_logger().info(
            f"Kinesis Controller Node #{self.turtle_num} is ready to go!"
        )

    def pose_callback(self, msg):
        """Callback for turtle pose."""
        moving_thresh = 0.001
        self.moving = (
            abs(msg.linear_velocity) > moving_thresh
            or abs(msg.angular_velocity) > moving_thresh
        )
        self.x = msg.x
        self.y = msg.y

    def reset_waiting(self):
        """Reset waiting for temperature."""
        self._waiting_for_temp = False
        self._temp_future = None

    def timer_callback(self):
        """Main loop for kinesis controller."""
        if self.moving:
            return

        # Get temperature at current position
        if self._waiting_for_temp and self._temp_future is not None:
            # We got a response while waiting for the service
            result = self._temp_future.result()
            if result is None:
                self.get_logger().warn("Temperature service call failed.")
                self.reset_waiting()
                return

            else:
                temperature = result.temperature
                self.reset_waiting()

        else:
            temp_req = GetTempByCoord.Request()
            temp_req.x = self.x
            temp_req.y = self.y
            self._temp_future = self.temp_client.call_async(temp_req)
            self._waiting_for_temp = True
            return

        # Map temperature to speed
        temp = max(temperature, 0.01)
        # linear velocity
        linear = random.choice([-1, 1]) * min(
            1 / temp * self.linear_vel_min, self.linear_vel_max
        )
        # angular velocity
        angular = random.choice([-1, 1]) * (
            min(1 / temp * self.angular_vel_min, self.angular_vel_max)
        )

        # Issue cmd_vel command
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        self.moving = True
        self.get_logger().info(
            f"Position ({self.x:.2f},{self.y:.2f}) Temp={temperature:.3f} "
            f"=> Linear={linear:.2f} Angular={angular:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = KinesisController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
