#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tb3_interfaces.action import NavGoal
import tf_transformations

#Non-ROS imports
import argparse, threading, time, yaml

class NavNode(Node):

    '''
    Constructor
        Inputs: name 
        Outputs: None
    '''
    def __init__(self, name, config):
        super().__init__(name+'_nav_node')

        self.get_logger().info(f'Started navigation node for robot {name}')

        #Parameters
        self.timer = self.get_clock().now()         #Timer for frequency of logging vicon topics
        self.thread_lock = threading.Lock()           #For locking parameters in multithreading

        #Initial pose (should come from vicon)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0

        #Motion parameters
        self.config = config                        
        self.P = self.config['PID']['P']
        self.diff_x = config['diff']['x']
        self.diff_y = config['diff']['y']
        self.diff_theta = config['diff']['theta']

        self.nav_action_server = ActionServer(
            self, NavGoal, 'nav_action',
            execute_callback=self.nav_execute_callback,
            goal_callback=self.nav_goal_callback,
            cancel_callback=self.nav_cancel_callback,
            callback_group=ReentrantCallbackGroup())
        
        self.vicon_sub = self.create_subscription(
            TransformStamped,
            f'vicon/{name}/{name}',
            self.vicon_pose_callback,
            10)

    '''
    Execute Callback: Execute the goal
        Inputs: goal_handle: ServerGoalHandle
        Outputs: None
    '''         
    def nav_execute_callback(self, goal_handle: ServerGoalHandle):
        
        #Get request from client
        goal_x = goal_handle.request.x
        goal_y = goal_handle.request.y
        goal_theta = goal_handle.request.theta

        #Initialize the action 
        self.get_logger().info(
            f'Executing the goal. Heading to {goal_x}, {goal_y}, {goal_theta}')
        feedback = NavGoal.Feedback()
        result = NavGoal.Result()

        #Calculate the difference between current and goal pose
        #Use dummy pose as example
        diff_x = goal_x - self.pose_x
        diff_y = goal_y - self.pose_y
        diff_theta = goal_theta - self.pose_theta
        self.get_logger().info(
            f'Initial differences: {diff_x, diff_y, diff_theta}')

        self.basic_P(feedback, diff_x, diff_y, diff_theta, goal_x, goal_y, goal_theta, goal_handle)

        #Set final goal state
        goal_handle.succeed()

        #Set and return the result
        result.x_final = self.pose_x
        result.y_final = self.pose_y
        result.theta_final = self.pose_theta
        self.get_logger().info(
            f'Reached final state: {result.x_final:.3f}, {result.y_final:.3f}, {result.theta_final:.3f}')

        return result


    '''
    Basic P: Basic P example for linear motion
        Inputs: feedback, goal_x, goal_y, goal_theta, goal_handle
        Outputs: None
    ''' 
    def basic_P(self, feedback, diff_x, diff_y, diff_theta, goal_x, goal_y, goal_theta, goal_handle):
        while not (abs(diff_x) < self.diff_x and abs(diff_y) < self.diff_y and abs(diff_theta) < self.diff_theta):

            #Update poses by adding P*difference, only if difference is still above threshold
            self.pose_x += self.P*diff_x*(abs(diff_x) > self.diff_x)
            self.pose_y += self.P*diff_y*(abs(diff_y) > self.diff_y)
            self.pose_theta += self.P*diff_theta*(abs(diff_theta) > self.diff_theta)

            #Update pose differential
            diff_x = goal_x - self.pose_x
            diff_y = goal_y - self.pose_y
            diff_theta = goal_theta - self.pose_theta

            #Send feedback
            feedback._x_delta = diff_x
            feedback._y_delta = diff_y
            feedback._theta_delta = diff_theta
            
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f'Current pose {self.pose_x:.3f}, {self.pose_y:.3f}, {self.pose_theta:.3f}')
            time.sleep(1)
        return

    '''
    Goal Callback: Accept/reject the goal?
        Inputs: goal_handle: ServerGoalHandle
        Outputs: None
    ''' 
    def nav_goal_callback(self, goal_handle: ServerGoalHandle):
        #POLICY: parralel goal execution (to start)
        self.get_logger().info("Received a goal.")
        self.get_logger().info("Accepting the goal.")
        return GoalResponse.ACCEPT
        
    def vicon_pose_callback(self, pose):

        trans = pose.transform.translation
        quat = pose.transform.rotation
        r, p, y = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        with self.thread_lock:
            self.pose_x = trans.x
            self.pose_y = trans.y
            self.pose_theta = r
        
        if (self.get_clock().now() - self.timer) > Duration(seconds=1.0):
            with self.thread_lock:
                self.get_logger().info(
                    f'Received vicon pose: ({trans.x:.2f}, {trans.y:.2f}, {trans.z:.2f})'\
                    f' ({r:.2f}, {p:.2f}, {y:.2f})')
                self.timer = self.get_clock().now()

    '''
    Cancel Callback: How to handle canceled goal?
        Inputs: goal_handle: ServerGoalHandle
        Outputs: None
    ''' 
    def nav_cancel_callback(self, goal_handle: ServerGoalHandle):
        #For now, nothing
        pass

    '''
    Handle Accept Callback: How to deal with accepted goal?
        Inputs: goal_handle: ServerGoalHandle
        Outputs: None
    '''  
    def nav_handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        #For now, nothing
        pass

def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", help="Name of robot", required=False, default="B04")
    parser.add_argument("--config", help="path to config file", 
        required=False, default="/home/glenn/ros2_ws/src/tb3_controller/config/motion_config.yaml")
    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()

    with open(args.config, 'r') as file:
        motion_config = yaml.safe_load(file)
        print(f"Motion config file found at: {args.config}")

    rclpy.init()
    node = NavNode(args.name, motion_config) 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()