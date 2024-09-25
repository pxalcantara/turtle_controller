#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dataclasses import dataclass

from std_msgs.msg import Bool
from turtle_controller.msg import RobotStatus
from turtle_controller.msg import RobotCmd

@dataclass
class Commands:
    direction: str
    velocity: float
    limit: float    

steps = [
    Commands(direction='front', velocity=0.5, limit=1.0),
    Commands(direction='left', velocity=0.5, limit=1.57),
    Commands(direction='front', velocity=0.5, limit=1.0),
]


class ControllerClient(Node):
    def __init__(self):
        super().__init__("controller_client")
        self.status_sub = self.create_subscription(RobotStatus, '/robot_status', self.status_callback, 10)
        self.start_sub = self.create_subscription(Bool, 'client_start', self.start_callback, 10)
        self.cmd_pub = self.create_publisher(RobotCmd, 'robot_cmd', 10)

        self.start_flag = False
        self.moving = False
        self.cmd_msg = RobotCmd()
        self.get_logger().info("This node just says 'Hello'")
        self.cmd_count = 0
    
    def start_callback(self, msg):
        self.start_flag = msg.data
        self.get_logger().info(f"Iniciando {msg.data}")
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
        if(not(self.start_flag)):
            return

        if (self.check_moving(msg)):
            return
        
          
        if (self.cmd_count == 0):
            self.move(steps[self.cmd_count].direction, steps[self.cmd_count].velocity, steps[self.cmd_count].limit)
            self.cmd_count += 1
        elif (self.check_command_finished(msg)):
            self.get_logger().info(f"Count {self.cmd_count}")
            self.move(steps[self.cmd_count].direction, steps[self.cmd_count].velocity, steps[self.cmd_count].limit)
            self.cmd_count += 1
        

        if (self.cmd_count == len(steps)):
            self.start_flag = False
            return            

        self.get_logger().info('Movendo')
            

def main(args=None):
    rclpy.init(args=args)
    node = ControllerClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()