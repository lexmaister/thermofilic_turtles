import rclpy
from rclpy.client import Client
from rclpy.node import Node

from turtlesim.srv import SetPen, TeleportAbsolute
from std_msgs.msg import Empty


def set_pen(
    node: Node,
    setpen_client: Client,
    r: int = 0,
    g: int = 0,
    b: int = 0,
    width=3,
    off=0,
    timeout_sec=1.0,
):
    """Set the turtle's pen color and width.

    Args:
        node (Node): The ROS2 node.
        setpen_client (Client): The service client for SetPen.
        r (int): Red color value (0-255).
        g (int): Green color value (0-255).
        b (int): Blue color value (0-255).
        width (int): Pen width.
        off (int): Pen state (0 for down, 1 for up).
        timeout_sec (float): Timeout for the service call.
    """
    req = SetPen.Request()
    req.r = r
    req.g = g
    req.b = b
    req.width = width
    req.off = off
    future = setpen_client.call_async(req)
    rclpy.spin_until_future_complete(node=node, future=future, timeout_sec=timeout_sec)
    if future.result() is None:
        node.get_logger().error("Failed to set pen")


def teleport(
    node: Node,
    teleport_client: Client,
    x: float = 0.0,
    y: float = 0.0,
    theta: float = 0.0,
    timeout_sec=1.0,
):
    """Teleport the turtle to a given position.

    Args:
        node (Node): The ROS2 node.
        teleport_client (Client): The service client for TeleportAbsolute.
        x (float): X coordinate.
        y (float): Y coordinate.
        theta (float): Orientation in radians.
        timeout_sec (float): Timeout for the service call.
    """
    req = TeleportAbsolute.Request()
    req.x = float(x)
    req.y = float(y)
    req.theta = float(theta)
    future = teleport_client.call_async(req)
    rclpy.spin_until_future_complete(node=node, future=future, timeout_sec=timeout_sec)
    if future.result() is None:
        node.get_logger().error("Failed to teleport")
