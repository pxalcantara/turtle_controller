#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from turtle_controller.msg import RobotStatus
from turtle_controller.msg import RobotCmd

class ControllerClient(Node):
    def __init__(self):
        super().__init__("controller_client")
        self.status_sub = self.create_subscription(RobotStatus, 'status', self.status_callback, 10)
        self.start_sub = self.create_subscription(Bool, 'client_start', self.start_callback, 10)
        self.cmd_pub = self.create_publisher(RobotCmd, 'robot_cmd', 10)

        self.start_flag = False
        self.once_cmd_flag = True
        self.cmd_msg = RobotCmd()
        self.get_logger().info("This node just says 'Hello'")
    
    def start_callback(self, msg):
        self.start_flag = msg.data

    def move(self, direction, velocity, limit):
        self.cmd_msg.direction = direction
        self.cmd_msg._velocity = velocity
        self.cmd_msg.limit = limit
        self.cmd_pub.publish(self.cmd_msg)

    def status_callback(self, msg):
        if (self.start_flag and self.once_cmd_flag):
            # self.get_logger().info(f"msg statua {msg}")
            self.move('back', 0.1, 1.0)
            self.get_logger().info('Movendo')
            self.once_cmd_flag = False

def main(args=None):
    rclpy.init(args=args)
    node = ControllerClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()