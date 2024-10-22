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
        self.cmd_pub = self.create_publisher(RobotCmd, 'robot_cmd', 10)

        self.start_flag = False
        self.moving = False
        self.cmd_msg = RobotCmd()
        self.get_logger().info("Client sensor!")
        self.cmd_count = 0
        self.prev = 'stop'

    
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

    def navigate_victor(self, msg, previous_direction=None):
        # EDITAR ESSA FUNÇÃO APENAS

        safe_distance = 0.2
        front_dist = msg.obstacle_distance.front
        left_dist = msg.obstacle_distance.left
        right_dist = msg.obstacle_distance.right

        max_distance = max(front_dist, left_dist, right_dist)
        self.get_logger().info(f"max: {max_distance}")

        if front_dist < safe_distance and left_dist < safe_distance and right_dist < safe_distance:
            self.move('stop', velocity=0.0, limit=0.0)
            self.get_logger().info("Cercado - girando 180 graus")
            self.move('left', velocity=0.5, limit=1.57)  # Gira 90 graus
            self.move('left', velocity=0.5, limit=1.57)  # Gira 90 graus

        elif max_distance == front_dist and previous_direction != 'front':
            self.move('front', velocity=0.5, limit=0.2)
            self.get_logger().info("Front")
            return 'front'

        elif max_distance == left_dist and previous_direction != 'left':
            self.move('left', velocity=0.5, limit=1.57)
            self.get_logger().info("Left")
            return 'left'

        elif max_distance == right_dist and previous_direction != 'right':
            self.move('right', velocity=0.5, limit=1.57)
            self.get_logger().info("Right")
            return 'right'

        else:
            self.move('stop', velocity=0.0, limit=0.0)
            self.get_logger().info("Stop")
            return 'stop'


    def status_callback(self, msg):

        if (self.check_moving(msg)):
            return
        
        self.prev = self.navigate_victor(msg, self.prev)
            

def main(args=None):
    rclpy.init(args=args)
    node = ControllerClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
