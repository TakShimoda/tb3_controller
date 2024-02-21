#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import ROSClock, ClockType
from rclpy.time  import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time as TimeStamp
from tb3_interfaces.action import NavGoal

#Non-ROS imports
import argparse, time, yaml

class NavClientNode(Node):

    '''
    Constructor
        Inputs: 
        Outputs: None
    '''
    def __init__(self, name, motion_config):
        super().__init__(name + 'nav_node')
        self.client = ActionClient(self, NavGoal, 'nav_action')
        self.get_logger().info(f'{name} nav client has initialized.')

        #Parameters
        self.name = name

    '''
    Send Goal
        Inputs: None 
        Outputs: None
    '''
    def send_nav_goal(self):
        goal_msg = NavGoal.Goal()
        goal_msg.x = 10.0
        goal_msg.y = 10.0
        goal_msg.theta = 5.0

        self.get_logger().info('Waiting for nav client...')
        self.client.wait_for_server()
        self.get_logger().info("Sending goal.")
        self.send_goal_future = self.client.send_goal_async(
            goal_msg, feedback_callback=self.nav_get_feedback_callback)
        self.send_goal_future.add_done_callback(self.nav_goal_response_callback)

    '''
    Goal Response Callback: Goal accepted/rejected?
        Inputs: future 
        Outputs: None
    '''
    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Planner has rejected goal for {self.name}!')
            return

        self.get_logger().info(f'Planner has accepted goal for {self.name}')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.nav_get_result_callback)

    '''
    Goal Feedback Callback: Feedback during goal execution
        Inputs: feedback_msg: Any 
        Outputs: None
    '''
    def nav_get_feedback_callback(self, feedback_msg):
        x_delta = feedback_msg.feedback.x_delta
        y_delta = feedback_msg.feedback.y_delta
        theta_delta = feedback_msg.feedback.theta_delta
        self.get_logger().info(
            f'Remaining pose difference: x: {x_delta:.3f} y: {y_delta:.3f} theta: {theta_delta:.3f}')

    '''
    Goal Result Callback: Goal result
        Inputs: future
        Outputs: None
    '''
    def nav_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'{self.name} has reached its goal.')

def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", help="robot name", required=False, default="B04")
    parser.add_argument("--config", help="path to config file", required=False, default="/home/glenn/ros2_ws/src/tb3_controller/config/motion_config.yaml")
    args = parser.parse_args()

    with open(args.config, 'r') as file:
        motion_config = yaml.safe_load(file)
        print(f"Motion config file found at: {args.config}")

    rclpy.init()
    action_client = NavClientNode(args.name, motion_config)
    action_client.send_nav_goal()
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


