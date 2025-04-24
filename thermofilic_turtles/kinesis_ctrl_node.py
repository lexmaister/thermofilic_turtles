#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class KinesisController(Node):
    def __init__(self):
        super().__init__("kinesis_controller")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("turtle_num", 1),
                ("linear_vel_min", 0.2),
                ("linear_vel_max", 3.0),
                ("angular_vel_min", 0.2),
                ("angular_vel_max", 3.14 / 2),
            ],
        )

        self.turtle_num = self.get_parameter("turtle_num").value
        self.linear_vel_min = self.get_parameter("linear_vel_min").value
        self.linear_vel_max = self.get_parameter("linear_vel_max").value
        self.angular_vel_min = self.get_parameter("angular_vel_min").value
        self.angular_vel_max = self.get_parameter("angular_vel_max").value

        self.moving = True  # if turtle isn't spawned yet
        self.field_lut = []  # field lookup table

        # Listeners
        self.create_subscription(Image, "temp_field", self.field_callback, 10)
        self.create_subscription(
            Pose, f"turtle{self.turtle_num}/pose", self.pose_callback, 10
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist, f"turtle{self.turtle_num}/cmd_vel", 10
        )

        # Publish ready message
        qos_profile = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.ready_pub = self.create_publisher(Empty, "ctrl_ready", qos_profile)
        self.ready_pub.publish(Empty())
        self.get_logger().info(
            f"Kinesis Controller Node #{self.turtle_num} is ready to go!"
        )

    def field_callback(self, msg):
        """Callback to handle new field info.

        Updates self.field_lut lookup table
        """
        self.get_logger().info(
            f"Got image: {msg.width}x{msg.height}, encoding={msg.encoding}, stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        )

        # Convert bytes to list of ints
        if msg.encoding == "mono8":
            # msg.data is a bytes object of length height*width (row-major order)
            flat = list(msg.data)
            height = msg.height
            width = msg.width

            # Reshape flat list into 2D list
            self.field_lut = [
                flat[i * width : (i + 1) * width] for i in range(height - 1, -1, -1)
            ]

            # Calculate mean value
            total = sum(flat)
            mean_val = total / (height * width) if height * width > 0 else 0

            self.get_logger().info(f"Image mean={mean_val:.2f}")

        else:
            self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")

    def pose_callback(self, msg):
        """Callback for turtle pose.

        If turtle isn't moving - sends new velocity command to /cmd_vel topic
        """
        moving_thresh = 0.001
        self.moving = (
            abs(msg.linear_velocity) > moving_thresh
            or abs(msg.angular_velocity) > moving_thresh
        )
        x = msg.x
        y = msg.y

        if self.moving or not self.field_lut:
            return

        coords = [0, 0]
        for i, coord in enumerate((x, y)):
            coords[i] = int(round(coord, 1) * 10)
            coords[i] = max(0, coords[i])
            coords[i] = min(109, coords[i])

        temp = self.field_lut[coords[1]][coords[0]]
        self.get_logger().debug(f"Temperature at point x={x:.1f}, y={y:.1f}: {temp}")

        vels = {"linear_vel": 0, "angular_vel": 0}
        for vel in ("linear_vel", "angular_vel"):
            vel_raw = (
                getattr(self, f"{vel}_min")
                + (256 - temp)
                * (getattr(self, f"{vel}_max") - getattr(self, f"{vel}_min"))
                / 256
            )
            vels[vel] = random.choice([-1, 1]) * min(
                vel_raw, getattr(self, f"{vel}_max")
            )

        # Issue cmd_vel command
        twist = Twist()
        twist.linear.x = vels["linear_vel"]
        twist.angular.z = vels["angular_vel"]
        self.cmd_vel_pub.publish(twist)
        self.moving = True
        self.get_logger().info(
            f"Position ({x:.1f},{y:.1f}) Temp={temp} "
            f"=> Linear={vels['linear_vel']:.2f} Angular={vels['angular_vel']:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = KinesisController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
