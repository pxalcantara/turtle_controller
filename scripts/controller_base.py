#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dataclasses import dataclass

from std_msgs.msg import Bool
from turtle_controller.msg import RobotStatus
from turtle_controller.msg import RobotCmd

class ControllerClient(Node):
    def __init__(self):
        super().__init__("controller_client")
        self.status_sub = self.create_subscription(RobotStatus, '/robot_status', self.status_callback, 10)
        self.cmd_pub = self.create_publisher(RobotCmd, 'robot_cmd', 10)

        self.moving = False
        self.cmd_msg = RobotCmd()
        self.get_logger().info("Client sensor!")
        self.cmd_count = 0

    
    def check_moving(self, msg):
        if (msg.moving == True):
            self.moving = True
            return True
        
        return False
    
    def check_command_finished(self, msg):
        if (self.moving and not(msg.moving)):
            return True
        
        return False


    def move(self, direction, velocity, limit):
        self.cmd_msg.direction = direction
        self.cmd_msg._velocity = velocity
        self.cmd_msg.limit = limit
        self.cmd_pub.publish(self.cmd_msg)

    def status_callback(self, msg):
        # Escreva o código nessa funcao, lembre-se, ela é executada em loop
        # Faz com que os comandos seguintes sejam ignorados se o robo estiver se movendo
        if (self.check_moving(msg)):
            return
        


def main(args=None):
    rclpy.init(args=args)
    node = ControllerClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()