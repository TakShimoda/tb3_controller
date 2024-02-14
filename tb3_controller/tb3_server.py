#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import TB

''''
    Node for representing computer
        Inputs:
            -num_robot: the number of robots connected
'''
class ComputerNode(Node): 
    def __init__(self, num_robots):
        super().__init__("computer") 
        self.num_robots = num_robots
        self.robot_count = 0 #count the number of robots we have communicated with
        self.server_ = self.create_service(TB, "TB_service", self.callback_response)
        self.get_logger().info("Computer server has been started.")
    
    def callback_response(self, request, response):
        self.robot_count+=1
        response.response = request.robot + " has been connected."
        self.get_logger().info(str(response.response))
        if self.robot_count==self.num_robots:
            self.start_robots()
        return response
    
    def start_robots(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ComputerNode(2) 
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()