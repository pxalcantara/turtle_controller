#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from turtle_controller.msg import RobotStatus

class ControllerClient(Node):
    def __init__(self):
        super().__init__("controller_client")
        self.subscription = self.create_subscription(RobotStatus, 'status', self.status_callback, 10)
        self.subscription
        self.get_logger().info("This node just says 'Hello'")

    def status_callback(self, msg):
        self.get_logger().info(f"msg statua {msg}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()