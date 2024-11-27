#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import ROSClock, ClockType
from rclpy.time  import Time
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as TimeStamp
from tb3_interfaces.action import ContactComputer, NavGoal
from tb3_interfaces.msg import RobotStatus
from tb3_interfaces.srv import ReturnPose
import tf_transformations

#Non-ROS imports
import argparse, math, threading, time, yaml
import numpy as np

class NavClientNode(Node):

    '''
    Constructor
        Inputs: 
        Outputs: None
    '''
    def __init__(self, name, client_config):
        super().__init__(name + '_nav_client_node')

        #Class members for communicating with server
        self.msg_logged = False                 #Already logged result from server? Only for purpose of last robot so it logs server result before subscription result
        self.log_msg = ''                       #Store log message in case it's the last robot.
        self.last_time = TimeStamp              #Time for last robot
        self.name = name                        #Name of robot

        #Motion parameters
        self.name = name
        self.client_config = client_config
        self.type = self.client_config['type']                                  #circular or square
        self.type_name = {0: 'circle', 1: 'square', 
                          2: 'square (smooth turns)', 3: 'angular'}[self.type]    
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0

        #Misc parameters
        self.thread_lock = threading.Lock()
        self.time = self.get_clock().now()

        #Service client for getting one-time pose from server
        self.pose_client = self.create_client(ReturnPose, name+'_pose')

        #Action client for initializing robot with computer
        self.comm_client = ActionClient(self, ContactComputer, 'group_trigger')
        #Navigation client
        self.nav_client = ActionClient(self, NavGoal, name+'_nav_action')
        self.get_logger().info(f'{name} nav client has initialized.')
        #Subscriber for counting total number of robots
        self.subscription = self.create_subscription(
            RobotStatus,
            'group_trigger_success',
            self.num_robots_callback,
            10)
        #Vicon subscriber
        self.vicon_sub = self.create_subscription(
            TransformStamped,
            f'vicon/{name}/{name}',
            self.vicon_pose_callback,
            10, 
            callback_group=MutuallyExclusiveCallbackGroup())
        
        
########################################################################################################################
#################### FUNCTIONS FOR COMMUNICATING WITH SERVER ####################
########################################################################################################################
    '''
    Send Goal: contact computer with robot name and current timestamp
        Inputs: None
        Outputs: None
    '''
    def send_msg(self):
        goal_msg = ContactComputer.Goal()
        goal_msg.robot = self.name
        clock = ROSClock()
        now = clock.now()
        goal_msg.timestamp.sec = now.seconds_nanoseconds()[0]
        goal_msg.timestamp.nanosec = now.seconds_nanoseconds()[1]

        self.comm_client.wait_for_server()
        self.send_goal_future = self.comm_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    '''
    Goal Response Callback: get computer's response if goal is accepted/rejected
        Inputs: 
            - future(future): holds the accept/reject result
        Outputs: None
    '''
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{self.name} failed to registered with computer!')
            return

        self.get_logger().info(f'{self.name} registered with computer')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    '''
    Goal Response Callback: get response from computer after first calling to it.
        Inputs: 
            - future(future): holds the final result, including the current number of robots
        Outputs: None
    '''
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'{self.name} contacted with computer as robot number {result.reached_number}.')
        self.msg_logged = True
        if not self.log_msg == '':
            self.get_logger().info(self.log_msg)
            self.set_timer(self.last_time)

    '''
    Number Robots Callback: callback to subscribe to topic broadcasting number of robots connected
        Inputs: 
            - robot_status: RobotStatus message containing (number of robots, latest time, total robots reached (T/F))
        Outputs: None
    '''
    def num_robots_callback(self, robot_status):
        final_msg = f'All {robot_status.num_robots} robots have been received by computer. Latest time is {robot_status.timestamp.sec}'
        # If total number of robots reached
        if robot_status.total_reached == True:
            # If last robot: this callback is triggered BEFORE the get_result_callback
            # Note this callback is received by every robot after every new robot reaches server
            # So if this is last robot, self.msg_logged isn't set yet (in the get_result_callback), 
            # and this will trigger the set_timer in the get_result_callback vy setting self.log_msg
            if not self.msg_logged:
                self.log_msg = final_msg
                self.last_time = robot_status.timestamp
            else:
                self.get_logger().info(final_msg)
                self.set_timer(robot_status.timestamp)
        else:
            self.get_logger().info(f'{robot_status.num_robots} robots currently received by computer. Latest time is {robot_status.timestamp.sec}')

    '''
    Set Timer: set the timer, arbitrarily x seconds after latest robot so all robots start moving together
        Inputs:
            - time_stamp(Time): the time of the last robot to call the computer.  
        Outputs:
    '''
    def set_timer(self, time_stamp):
        self.get_logger().info('Starting timer... ')
        last_time = Time(seconds=time_stamp.sec, nanoseconds=time_stamp.nanosec, clock_type=ClockType.ROS_TIME)
        start_time = last_time + Duration(seconds=2)
        delay = start_time - self.get_clock().now()

        if delay < Duration(seconds=0):
            self.get_logger().warn('Timer is negative. Resetting it to be positive')
            delay = Duration(seconds=2)
        
        self.timer = self.create_timer(delay.nanoseconds/1e9, self.motion_planner)
        self.get_logger().info(f'Timer started. Starting in {delay.nanoseconds/1e9} seconds.')

########################################################################################################################
#################### MOTION FUNCTIONS ####################
########################################################################################################################

    '''
    Motion planner: plans all motion, including initial spinning, from beginning to end of program
        Inputs: None 
        Outputs: None
    '''
    def motion_planner(self):
        current_goal = 0
        #Then send the specified motion
        #current_goal = self.send_all_nav_goals(current_goal, 1)
        self.get_logger().info('timer cancelled...')
        self.timer.cancel()
        self.navigate()

    '''
    Vicon Pose Callback: Get the vicon pose (copied from server)
        Inputs: pose(TransformStamped) - in vicon coordinates (x-forward, y-left)
        Outputs: None
    ''' 
    def vicon_pose_callback(self, pose):

        trans = pose.transform.translation
        quat = pose.transform.rotation
        r, p, y = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        with self.thread_lock:
            self.pose_x = trans.x
            self.pose_y = trans.y
            self.pose_theta = y
        
        if (self.get_clock().now() - self.time) > Duration(seconds=1.0):
            with self.thread_lock:
                self.get_logger().info(
                    f'Received vicon pose (x, y, theta(deg)): ({trans.x:.3f}, {trans.y:.3f}, {y*180/math.pi:.3f})')
                self.time = self.get_clock().now()
    
    '''
    Send all navigation goals
        Inputs: 
            - type: circular, linear, or angular 
            - current_goal: the current goal ID in order to stack goal ID of different motions
            - repeat: how many times to perform the motion (1 for no repeat, 2 for doing it twice, etc...)
        Outputs: None
    '''
    # def send_all_nav_goals(self, current_goal):
    #     # goals_queue = self.create_all_goals(type_)
    #     goals_queue = self.create_all_goals()
    #     #repeat n times
    #     goals_queue = goals_queue
    #     for goal_id, goals in enumerate(goals_queue):
    #         for wp_id, waypoints in enumerate(goals):
    #             self.send_nav_goal(waypoints, goal_id+current_goal, wp_id)
    #             #sleep to prevent first two goals executing at once
    #             time.sleep(0.05)
    #         self.get_logger().info(f'Finished sending goal {goal_id}')
    #     self.get_logger().info(f'Finished sending all goals for {self.type_name} motion.')
    #     return len(goals_queue)
    '''
    Navigate: simple navigation for now w/o action involved
        Inputs: None
        Outputs: None
    '''
    def navigate(self):
        goal_x = 2.0
        goal_y = 2.0
        goal_theta = 0.0
        self.get_logger().info(f'Goal is {goal_x}, {goal_y}, {goal_theta}.')
        self.get_logger().info(
            f'Current robot position is {self.pose_x:.3f}, {self.pose_y:.3f}, {self.pose_theta:.3f}.')
        self.get_logger().info(
            f'Difference is {(goal_x-self.pose_x):.3f}, {(goal_y-self.pose_x):.3f}, {(goal_theta-self.pose_theta):.3f}.')
    
        #first fix heading by calculating angle
        theta = np.arctan(goal_x/goal_y)
