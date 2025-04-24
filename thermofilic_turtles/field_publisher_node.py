#!/usr/bin/env python3

import random
import struct
import os

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult


pkg_name = "thermofilic_turtles"
pkg_share = get_package_share_directory(pkg_name)


def load_bmp_image(filename: str) -> tuple[tuple]:
    """Load a BMP image as a 2D tuple of grayscale uint8 values (0-255).
    Supports 8-bit (paletted) and 24-bit BMPs.

    Args:
        filename (str) - path to the image file

    Returns:
        tuple[tuple] - image pixels data
    """
    with open(filename, "rb") as f:
        header = f.read(54)
        assert header[:2] == b"BM", "Not a BMP file"
        offset = struct.unpack("<I", header[10:14])[0]
        width = struct.unpack("<I", header[18:22])[0]
        height = struct.unpack("<I", header[22:26])[0]
        bpp = struct.unpack("<H", header[28:30])[0]

        r_coeff = 0.299
        g_coeff = 0.587
        b_coeff = 0.114
        if bpp == 8:
            # Read palette: 256 * 4 bytes (B,G,R,0)
            palette_size = 256 * 4
            palette_bytes = f.read(palette_size)
            palette = [
                (palette_bytes[i + 2], palette_bytes[i + 1], palette_bytes[i])
                for i in range(0, palette_size, 4)
            ]
            row_size = (width + 3) & ~3
            f.seek(offset)
            img = []
            for _ in range(height):
                row_data = f.read(row_size)[:width]  # ignore padding
                row = []
                for index in row_data:
                    r, g, b = palette[index]
                    gray = int(r_coeff * r + g_coeff * g + b_coeff * b)
                    row.append(gray)
                img.append(tuple(row))
        elif bpp == 24:
            row_size = (width * 3 + 3) & ~3
            f.seek(offset)
            img = []
            for _ in range(height):
                row_data = f.read(row_size)
                row = []
                for x in range(width):
                    i = x * 3
                    b, g, r = row_data[i : i + 3]
                    gray = int(r_coeff * r + g_coeff * g + b_coeff * b)
                    row.append(gray)
                img.append(tuple(row))
        else:
            raise NotImplementedError(
                f"BMP bpp {bpp} not supported. Use 8 or 24 bpp BMPs."
            )
        img.reverse()  # BMP images are stored bottom-to-top
        return tuple(img)


class FieldPubliserNode(Node):

    def __init__(self):
        super().__init__("field_publisher")
        self.get_logger().info("Field Publisher Node has been started")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("img_file", "smiling_face.bmp"),
                ("timer_period", 1.0),
                ("noise_stddev", 10),
            ],
        )

        img_file = self.get_parameter("img_file").value
        timer_period = self.get_parameter("timer_period").value
        self.noise_stddev = self.get_parameter("noise_stddev").value

        self.add_on_set_parameters_callback(self.param_callback)

        img_path = os.path.join(pkg_share, "resource", img_file)
        self.get_logger().info(f"Loading field img from: {img_path!r}")
        self.img = load_bmp_image(img_path)
        self.get_logger().info(
            f"Loaded field img {img_file!r}, rows: {len(self.img)}, cols: {len(self.img[0])}"
        )

        # Publisher - temp field
        self.field_pub = self.create_publisher(Image, f"temp_field", 10)

        # Timer to publish periodically
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Field Publisher Node is ready")

    def param_callback(self, params):
        """Handle dynamic parameter updates."""
        successful = True
        for param in params:
            if param.name == "noise_stddev" and param.type_ == param.Type.INTEGER:
                self.noise_stddev = param.value
                self.get_logger().info(f"noise_stddev updated to {self.noise_stddev}")
            elif param.name == "img_file" and param.type_ == param.Type.STRING:
                # Attempt to reload image
                try:
                    img_path = os.path.join(pkg_share, "resource", param.value)
                    self.img = load_bmp_image(img_path)
                    self.get_logger().info(f"Reloaded image: {param.value}")
                except Exception as e:
                    self.get_logger().error(f"Failed to load {param.value}: {e}")
                    successful = False
            elif param.name == "timer_period" and param.type_ == param.Type.DOUBLE:
                try:
                    self.timer.cancel()
                    self.timer = self.create_timer(param.value, self.timer_callback)
                    self.get_logger().info(f"Timer period updated to {param.value}")
                except Exception as e:
                    self.get_logger().error(f"Failed to update timer period: {e}")
                    successful = False
        return SetParametersResult(successful=successful)

    def timer_callback(self):
        """Publish field info as image"""
        msg = self.make_image_msg()
        self.field_pub.publish(msg)
        self.get_logger().info("Published temperature field image.")

    def make_image_msg(self):
        """Construct a sensor_msgs/Image message from loaded grayscale image, with optional noise."""
        height = len(self.img)
        width = len(self.img[0])
        data = []
        for row in self.img:
            for val in row:
                v = int(val + random.gauss(0, self.noise_stddev))
                v = max(0, min(255, v))
                data.append(v)
        assert (
            len(data) == height * width
        ), f"Data length {len(data)} doesn't match {height}x{width}"
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.height = height
        msg.width = width
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = width
        msg.data = bytearray(data)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = FieldPubliserNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
