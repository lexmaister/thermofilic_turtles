#!/usr/bin/env python3

import random
import math

import rclpy
from rclpy.node import Node

from turtlesim.srv import SetPen, TeleportAbsolute

from turtlesim_utils import set_pen, teleport


def temp_to_rgb(temp):
    """Returns an RGB color tuple for a given temperature value between 0 and 1 in a blue to red scale."""
    r = int(255 * temp)
    g = 0
    b = int(255 * (1 - temp))
    return r, g, b


class GradientPainter(Node):
    """A ROS2 node that paints a gradient on the turtlesim screen."""

    def __init__(self):
        super().__init__("gradient_painter")
        self.get_logger().info("Gradient Painter Node has been started")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("heat_center_x", 4.0),
                ("heat_center_y", 3.0),
                ("heat_sigma", 2.5),
                ("isoline_step", 0.1),
            ],
        )

        self.isoline_step = self.get_parameter("isoline_step").value
        self.heat_center_x = self.get_parameter("heat_center_x").value
        self.heat_center_y = self.get_parameter("heat_center_y").value
        self.heat_sigma = self.get_parameter("heat_sigma").value

        self.pen_client = self.create_client(SetPen, "turtle1/set_pen")
        self.teleport_client = self.create_client(
            TeleportAbsolute, "turtle1/teleport_absolute"
        )

        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_pen service...")
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for teleport_absolute service...")

        self.paint_isolines()

    def temperature(self, x, y):
        """Gaussian temperature function."""
        res = math.exp(
            -((x - self.heat_center_x) ** 2 + (y - self.heat_center_y) ** 2)
            / (2 * self.heat_sigma**2)
        )
        return res

    def paint_isolines(self):
        """Paint isolines on the turtlesim background."""
        self.get_logger().info("Painting isolines")
        sigma = self.heat_sigma
        x_c = self.heat_center_x
        y_c = self.heat_center_y
        n_points = 120  # Number of points per circle

        # Define temperature levels (isoline values)
        temp_min = 0.1  # Start above zero to avoid math issues
        step = self.isoline_step
        T_values = [i * step for i in range(int(temp_min / step), int(1.0 / step) + 1)]

        for T_i in T_values:
            # Skip invalid temperature values
            if T_i <= 0 or T_i >= 1:
                continue

            self.get_logger().info(f"Painting isoline for T={T_i:.2f}")
            # Calculate radius and color for the isoline
            r = sigma * math.sqrt(-2 * math.log(T_i))
            r_g_b = temp_to_rgb(T_i)

            # Move pen up and teleport to the start of the circle
            theta0 = 0
            x0 = x_c + r * math.cos(theta0)
            y0 = y_c + r * math.sin(theta0)
            set_pen(self, self.pen_client, off=1)
            teleport(self, self.teleport_client, x0, y0)
            set_pen(self, self.pen_client, *r_g_b, width=2, off=0)

            # Draw the circle by incrementing the angle
            for i in range(1, n_points + 1):
                theta = 2 * math.pi * i / n_points
                x = x_c + r * math.cos(theta)
                y = y_c + r * math.sin(theta)
                teleport(self, self.teleport_client, x, y)
            # Optionally, close the circle by returning to (x0, y0)
            teleport(self, self.teleport_client, x0, y0)

        # Turn off the pen after drawing and teleport randomly
        set_pen(self, self.pen_client, off=1)
        teleport(
            self, self.teleport_client, random.uniform(1, 10), random.uniform(1, 10)
        )
        self.get_logger().info("Gradient painting completed.")


def main(args=None):
    rclpy.init(args=args)
    node = GradientPainter()
    rclpy.spin_once(node, timeout_sec=0)  # Just initialize and paint!
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
