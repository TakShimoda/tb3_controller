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
import argparse, math, time, yaml
import numpy as np

'''
TODO: Make certain functions not node functions but simple functions:
    - create_waypoints, create_goal
    - these functions don't need self
'''

class NavClientNode(Node):

    '''
    Constructor
        Inputs: 
        Outputs: None
    '''
    def __init__(self, name, client_config):
        super().__init__(name + 'nav_node')
        self.client = ActionClient(self, NavGoal, 'nav_action')
        self.get_logger().info(f'{name} nav client has initialized.')

        #Parameters
        self.name = name
        self.client_config = client_config

    '''
    Create goal: create goal in global coordinates
        Inputs: 
        - type: (circ/linear/angular)
        - current pose: (x, y s0_2). so_2 is used so we don't have to convert to get angles
        Outputs: goal in global coordinates
    '''
    def create_goal(self, type, dist_lin, dist_theta, x, y, SO_2):
        if type == 'circular':
        #use polar coordinates
            radius = dist_lin/dist_theta
            theta_loc = dist_theta
            x_loc = radius*np.cos(theta_loc) - radius
            y_loc = radius*np.sin(theta_loc)
        elif type == 'linear':
            x_loc = 0.0
            y_loc = dist_lin
            theta_loc = 0.0
        else: #turning/angular
            x_loc = 0.0
            y_loc = 0.0
            theta_loc = dist_theta

        #Define goal in local coordinates
        goal_local = np.array([[np.cos(theta_loc), -np.sin(theta_loc), x_loc], 
                                [np.sin(theta_loc), np.cos(theta_loc), y_loc], 
                                [0, 0, 1]])
        robot_pose = np.array([[SO_2[0,0], SO_2[0,1], x], 
                                [SO_2[1,0], SO_2[1,1], y], 
                                [0, 0, 1]])  
        #Define goal in global coordinates
        goal_global = robot_pose@goal_local

        return goal_global

    '''
    Create waypoints: create equidistant waypoints each as goals for server to complete
        - Moreover, this would take a larger goal (e.g. quarter circle) and break it down to smaller goals
        - Also, the global goal returned on one iteration can serve as the pose in the next iteration 
            to easily make consecutive equidistant waypoints
        Inputs: 
            - type: circular, linear, or other(i.e. angular)
            - total_linear: total linear distance
            - total_theta: total angular distance
            - num_points: number of points
            - (x, y, theta): current robot pose - will have to be received via a simple server or one-time subscription
        Outputs: None
    '''
    def create_waypoints(self, type, total_linear, total_theta, num_points, x, y, theta):
        waypoints = []
        dist_lin = total_linear/num_points
        dist_theta = total_theta/num_points 
        #Create initial SO(2)
        SO_2 = np.array([[np.cos(theta), -np.sin(theta)], 
                        [np.sin(theta), np.cos(theta)]])
        for i in range(num_points):
            #calculate global points (x, y, theta)
            goal_global = self.create_goal(type, dist_lin, dist_theta, x, y, SO_2)
            waypoints.append(goal_global)
            x = goal_global[0, 2]
            y = goal_global[1, 2]
            SO_2 = goal_global[0:2, 0:2]

        return waypoints

    '''
    Create all goals: create all subgoals and their respective waypoints
        - get the robot pose (with a service asking for the pose one time)
        - iterate through all the sub-goals (e.g. quarter circle), get waypoints for each, and execute each subgoal
        Inputs: None 
        Outputs: None
    '''
    def create_all_goals(self):
        type_ = self.client_config['type']
        if type_ == 'circular':
            dist_lin =  self.client_config['circ']['rad']*self.client_config['circ']['angle']*math.pi/180.0
            dist_theta = self.client_config['circ']['angle']
            num_goals = 360//dist_theta #number of subarcs to take
        elif type_ == 'linear':
            dist_lin = self.client_config['square']['dist']
            dist_theta = 0.0
            num_goals = 4
        else: #angular
            dist_lin = 0.0
            dist_theta = self.client_config['turn']['angle']
            num_goals = 360//dist_theta
            
        #Get poses and properly iterate to get waypoints!
        x, y, theta = 0, 0, 0
        waypoints = self.create_waypoints(type_, dist_lin, dist_theta, x, y, theta)

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
    parser.add_argument("--config", help="path to config file", required=False, 
                        default="/home/glenn/ros2_ws/src/tb3_controller/config/client_config.yaml")
    args = parser.parse_args()

    with open(args.config, 'r') as file:
        motion_config = yaml.safe_load(file)
        print(f"Client config file found at: {args.config}")

    rclpy.init()
    action_client = NavClientNode(args.name, motion_config)
    action_client.send_nav_goal()
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


