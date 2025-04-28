#!/usr/bin/env python3

import random
import math

import rclpy
from geometry_msgs.msg import Twist
from thermofilic_turtles.kinesis_ctrl_node import KinesisController


class TaxisController(KinesisController):
    def __init__(self):
        super().__init__(ctrl_type="taxis")

        self.declare_parameter("taxis_base", 0.2)
        self.taxis_base = self.get_parameter("taxis_base").value

    def map_coord(self, coord):
        """Map coordinate to LUT index."""
        coord = int(round(coord, 1) * 10)
        coord = max(0, coord)
        coord = min(109, coord)
        return coord

    def pose_callback(self, msg):
        """Callback for turtle pose - calc gradient on taxis base.

        If turtle isn't moving - sends new velocity command to /cmd_vel topic
        """
        moving_thresh = 0.001
        self.moving = (
            abs(msg.linear_velocity) > moving_thresh
            or abs(msg.angular_velocity) > moving_thresh
        )
        x = msg.x
        y = msg.y
        theta = msg.theta
        self.get_logger().debug(f"Pose: x={x:.1f}, y={y:.1f}, theta={theta:.1f} ")

        if self.moving or not self.field_lut:
            return

        center = [0, 0]
        for i, coord in enumerate((x, y)):
            center[i] = self.map_coord(coord)

        temp_center = self.field_lut[center[1]][center[0]]
        self.get_logger().debug(
            f"Temperature at center x={center[0]:.1f}, y={center[1]:.1f}: {temp_center}"
        )

        outer = [0, 0]
        for i, coord in enumerate(
            (
                x + self.taxis_base * math.cos(theta),
                y + self.taxis_base * math.sin(theta),
            )
        ):
            outer[i] = self.map_coord(coord)

        temp_outer = self.field_lut[outer[1]][outer[0]]
        self.get_logger().debug(
            f"Temperature at outer x={outer[0]:.1f}, y={outer[1]:.1f}: {temp_outer}"
        )

        # Calculate temperature gradient
        temp_gradient = temp_outer - temp_center
        temp_gradient = (
            temp_gradient if abs(temp_gradient) > 0.1 else random.uniform(-0.1, 0.1)
        )
        self.get_logger().debug(f"Temperature gradient: {temp_gradient:.1f}")

        # Determine desired direction: forward or backward
        # If the gradient is positive, move toward it; if negative, move away
        if temp_gradient > 0:
            move_dir = 1.0  # Turn toward gradient
        else:
            move_dir = -1.0  # Turn away from gradient (negative taxis or avoidance)

        # Modulate forward velocity by gradient's magnitude
        vel_gain = min(abs(temp_gradient), 1.0) * (
            self.linear_vel_max - self.linear_vel_min
        )
        linear = move_dir * min(self.linear_vel_min + vel_gain, self.linear_vel_max)

        # Calculate preferred angle (theta_desired)
        theta_vector = math.atan2(outer[1] - center[1], outer[0] - center[0])
        dtheta = theta_vector - theta
        dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        # Modulate angular velocity by gradient's magnitude
        angular_vel_gain = min(abs(temp_gradient), 1.0) * (
            self.angular_vel_max - self.angular_vel_min
        )
        if abs(temp_gradient) > 0.1:
            angular = angular_vel_gain * dtheta
        else:
            angular = random.uniform(-self.angular_vel_max, self.angular_vel_max)

        # Issue cmd_vel command
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        self.moving = True
        self.get_logger().info(
            f"Position ({x:.1f},{y:.1f}) Gradient={temp_gradient} "
            f"=> Linear={linear:.2f} Angular={angular:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TaxisController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
