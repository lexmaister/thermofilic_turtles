#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, SetPen

from turtlesim_utils import set_pen


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.get_logger().info("Turtle Spawner Node has been started")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("turtles_count", 5),
            ],
        )
        self.turtles_count = self.get_parameter("turtles_count").value

        self.spawn_client = self.create_client(Spawn, "spawn")
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /spawn service...")

        for i in range(2, self.turtles_count + 1):
            x = random.uniform(0.5, 10.5)
            y = random.uniform(0.5, 10.5)
            theta = i * (360 / self.turtles_count)
            name = f"turtle{i}"
            self.get_logger().info(f"Spawning {name} at ({x}, {y})")
            self.spawn_turtle(x, y, theta, name)

        self.get_logger().info("All turtles spawned.")

    def spawn_turtle(self, x, y, theta, name):
        req = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = theta
        req.name = name
        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            self.get_logger().info(f"Spawned turtle: {future.result().name}")
        else:
            self.get_logger().error("Failed to spawn turtle")

        self.get_logger().info(f"Setting pen for {name}")
        self.pen_client = self.create_client(SetPen, f"{name}/set_pen")
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {name}/set_pen service...")

        set_pen(self, self.pen_client, off=1)
        self.get_logger().info(f"Pen is disabled for {name}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin_once(node, timeout_sec=0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
