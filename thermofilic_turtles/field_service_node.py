#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from thermofilic_turtle.srv import GetTempByCoord


class FieldServiceNode(Node):

    def __init__(self):
        super().__init__("field_service")
        self.get_logger().info("Field Service Node has been started")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("heat_center_x", 4.0),
                ("heat_center_y", 3.0),
                ("heat_sigma", 2.0),
                ("noise_stddev", 0.0),
            ],
        )

        self.heat_center_x = self.get_parameter("heat_center_x").value
        self.heat_center_y = self.get_parameter("heat_center_y").value
        self.heat_sigma = self.get_parameter("heat_sigma").value
        self.noise_stddev = self.get_parameter("noise_stddev").value

        self.factor = 2 * self.heat_sigma**2
        self.get_logger().info(
            f"FieldServiceNode initialized with center ({self.heat_center_x}, {self.heat_center_y}), "
            f"sigma {self.heat_sigma}, noise stddev {self.noise_stddev}"
        )

        self.callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            GetTempByCoord,
            "get_temp_by_coord",
            self.compute_temp_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info("FieldServiceNode ready with parameters.")

    def compute_temp_callback(self, request, response):
        # Minimal computation for performance
        x = request.x
        y = request.y
        dx = x - self.heat_center_x
        dy = y - self.heat_center_y
        r2 = dx * dx + dy * dy

        gaussian = math.exp(-r2 / self.factor)
        noise = random.gauss(0, self.noise_stddev)
        response.temperature = gaussian + noise

        # Reducing logging frequency to avoid flooding
        self.get_logger().debug(
            f"Temp at ({x:.2f}, {y:.2f}) = {response.temperature:.3f}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FieldServiceNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
