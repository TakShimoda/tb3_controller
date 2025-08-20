#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import ROSClock, ClockType
from rclpy.time  import Time
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as TimeStamp
from tb3_interfaces.action import ContactComputer, NavGoal
from tb3_interfaces.msg import RobotStatus
from tb3_interfaces.srv import ReturnPose
import tf_transformations

from tb3_controller.motion_func import create_waypoints

#Non-ROS imports
import argparse, math, threading, time, yaml
import numpy as np
import pandas as pd

'''
TODO: Make certain functions not node functions but simple functions:
    - create_waypoints, create_goal
    - these functions don't need self
    - right now waypoints are a list of tuples [SE(2) vector, linear distance, angular distance]
        - linear and angular distance are repetitive and the same for circles/turning, so they should be
            calculated just once. 
        - similar efficiencies can be gained for square, but it still involves straight+turning motion
    - make simple way of programatically creating arbitrary waypoint patterns and incorporating into create_all_goals
        - right now, create_all_goals only works on assumption of circle/square/turning
'''

class NavClientNode(Node):

    '''
    Constructor
        Inputs: 
        Outputs: None
    '''
    def __init__(self, name, client_config, paths_csv):
        super().__init__(name + '_nav_client_node')

        #Class members for communicating with server
        self.msg_logged = False                 #Already logged result from server? Only for purpose of last robot so it logs server result before subscription result
        self.log_msg = ''                       #Store log message in case it's the last robot.
        self.last_time = TimeStamp              #Time for last robot
        self.name = name                        #Name of robot

        #csv of paths
        self.paths_csv = paths_csv
        #self.create_goals_from_csv()

        #Motion parameters
        self.name = name
        self.client_config = client_config
        self.type = self.client_config['type']                                  #circular or square
        self.type_name = {0: 'circle', 1: 'square', 
                          2: 'square (smooth turns)', 3: 'angular'}[self.type]    
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        self.repeat = self.client_config['repeat']
        self.use_csv = self.client_config['use_csv']

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
            10)
        
########################################################################################################################
#################### FUNCTIONS FOR COMMUNICATING WITH SERVER ####################
########################################################################################################################
    '''
    Send Goal: contact computer with robot name and current timestamp
        Inputs: None
        Outputs: None
    '''
    def send_goal(self):
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

    def create_goals_from_csv(self):
        df = pd.read_csv(self.paths_csv, header=0)
        # initial poses
        with self.thread_lock:
            x = -self.pose_y
            y = self.pose_x
            theta = self.pose_theta
        # x, y, theta = 0, 0, 0
        dist_lin = 0
        num_waypoints = 1
        goals_queue = []
        for _, row in df.iterrows():
            motion_type = {0: 'linear', 1: 'angular', 2: 'circular'}[row['type']]
            dist_theta = row['theta']*math.pi/180.0
            if motion_type in ('linear', 'angular'):
                dist_lin = row['x']
            else:
                dist_lin = abs(dist_theta)*row['x']
            waypoints = create_waypoints(motion_type, dist_lin, dist_theta, num_waypoints, x, y, theta)
            #print(waypoints[-1][0].reshape(3, 3))

            #Set x,y to the last waypoint (last waypoint entry(-1), SE(2) matrix [0], and then x, y [2] and [5])
            x, y = waypoints[-1][0][2], waypoints[-1][0][5]
            #Update theta to what it would be at the end of the waypoints
            theta += dist_theta
            theta -= (2*math.pi*(theta>math.pi))
            goals_queue.append(waypoints)
        return goals_queue
    '''
    Create all goals: create all subgoals and their respective waypoints
        - get the robot pose (with a service asking for the pose one time)
        - iterate through all the sub-goals (e.g. quarter circle), get waypoints for each, and execute each subgoal
        Inputs: None 
        Outputs: goals queue: [waypoints]
    '''
    def create_all_goals(self):
        #Initialize parameters
        if self.type == 0: #'circular'
            dist_theta = self.client_config['circ']['angle']*math.pi/180.0
            dist_lin =  self.client_config['circ']['rad']*dist_theta
            #If distance is less/equal to 360 degrees
            if dist_theta <= math.pi:
                num_goals = int(2*math.pi//dist_theta) #number of subarcs to take
            else:
                num_goals = 1
            num_waypoints = self.client_config['circ']['num_points']
        elif self.type in (1, 2): #==2,3 'square'
            dist_lin = self.client_config['square']['dist']
            dist_theta = 0.0
            num_goals = 4
            num_waypoints = self.client_config['square']['num_points']
            if self.type == 2: #smooth motion
                dist_circ_lin = self.client_config['circ_turn']['rad']*self.client_config['circ_turn']['angle']*math.pi/180.0
        else: #angular
            #Turn parameters
            dist_lin = 0.0
            dist_theta = self.client_config['turn']['angle']*math.pi/180.0
            num_goals = int(2*math.pi//dist_theta)
            num_waypoints = self.client_config['turn']['num_points']
            
        self.get_logger().info(f'There are {num_goals} goals and {num_waypoints} waypoints for each.')
        self.get_logger().info("Waiting to get current pose to start planning waypoints ...")
        self.total_waypoints_goal = num_goals*num_waypoints

        # current_pose = self.get_pose()
 
        # #We get poses in vicon coordinates. Convert to Cartesian coordinates (theta remains the same)
        # x = -current_pose.y
        # y = current_pose.x
        # theta = current_pose.theta

        #New method for pose: use vicon subscriber on client side to get pose
        with self.thread_lock:
            x = -self.pose_y
            y = self.pose_x
            theta = self.pose_theta

        #Iterate over all subgoals to create all goals
        goals_queue = []
        for i in range(num_goals):
            self.get_logger().info(f'=====Making goal number {i}.' 
                    f' Current robot pose (Cartesian): x= {x:.3f}, y={y:.3f}, theta={theta*180.0/math.pi:.3f}.=====')

            if self.type in (1, 2): #== 2, 3: #'square'
            #Straight motion
                waypoints = create_waypoints('linear', dist_lin, dist_theta, num_waypoints, x, y, theta)
                goals_queue.append(waypoints)
                #Set x,y to the last waypoint (last waypoint entry(-1), SE(2) matrix [0], and then x, y [2] and [5])
                x, y = waypoints[-1][0][2], waypoints[-1][0][5]
                #Update theta to what it would be at the end of the waypoints
                theta += dist_theta
                theta -= (2*math.pi*(theta>math.pi))

            #Turn motion
                if self.type == 1: #sharp turns
                    waypoints = create_waypoints('angular', 0.0, math.pi/2, 1, x, y, theta)
                    goals_queue.append(waypoints)
                    x, y = waypoints[-1][0][2], waypoints[-1][0][5]
                    theta += math.pi/(2*num_waypoints)
                    theta -= (2*math.pi*(theta>math.pi))
                else: #smooth turns
                    waypoints = create_waypoints('circular', dist_circ_lin, math.pi/2, 1, x, y, theta)
                    goals_queue.append(waypoints)
                    x, y = waypoints[-1][0][2], waypoints[-1][0][5]
                    theta += math.pi/(2*num_waypoints)
                    theta -= (2*math.pi*(theta>math.pi))
            #circular/angular        
            else:
                waypoints = create_waypoints(self.type, dist_lin, dist_theta, num_waypoints, x, y, theta)
                goals_queue.append(waypoints)
                #Reset the pose to make it equal to the goal pose, to feed into next iteration
                x, y = waypoints[-1][0][2], waypoints[-1][0][5]
                theta += dist_theta - (2*math.pi*(theta>2*math.pi))
                theta -= (2*math.pi*(theta>math.pi))
        return goals_queue

    '''
    Get pose: contact that server for pose once to start waypoint planning
        Inputs: None 
        Outputs: ReturnPose.response (x, y, theta) in vicon coordinates
    '''
    def get_pose(self):
        while not self.pose_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for pose...")
        request = ReturnPose.Request()
        request.topic = f'vicon/{self.name}/{self.name}'

        #Spin until we get the pose
        future = self.pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    '''
    Motion planner: plans all motion, including initial spinning, from beginning to end of program
        Inputs: None 
        Outputs: None
    '''
    def motion_planner(self):
        #First start with spinning motion, get the last goal number to stack up the goal queue at once
        current_goal = 0
        #current_goal = self.send_all_nav_goals('angular', current_goal)
        #Then send the specified motion
        current_goal = self.send_all_nav_goals(current_goal)
        self.timer.cancel()
       # self.get_logger().info(f'Completed all motion. {self.name} finished.')

    '''
    Send all navigation goals
        Inputs: 
            - current_goal: the current goal ID in order to stack goal ID of different motions
            - repeat: how many times to perform the motion (1 for no repeat, 2 for doing it twice, etc...)
        Outputs: int (length of goal queue)
    '''
    def send_all_nav_goals(self, current_goal)->int:
        # goals_queue = self.create_all_goals(type_)
        if self.use_csv:
            goals_queue = self.create_goals_from_csv()
        else:
            goals_queue = self.create_all_goals()
        #repeat n times
        goals_queue = goals_queue*self.repeat
        self.get_logger().info(f'Repeating trajectory {self.repeat} times.')
        for goal_id, goals in enumerate(goals_queue):
            for wp_id, waypoints in enumerate(goals):
                self.send_nav_goal(waypoints, goal_id+current_goal, wp_id)
                #sleep to prevent first two goals executing at once
                time.sleep(0.08)
            self.get_logger().info(f'Finished sending goal {goal_id}')
        self.get_logger().info(f'Finished sending all goals for {self.type_name} motion.')
        return len(goals_queue)
        
    '''
    Send Goal
        Inputs: 
        - waypoint (se(2) vector, x local, theta local, theta global)
        - goal_id
        - wp_id 
        Outputs: None
    '''
    def send_nav_goal(self, waypoint, goal_id, wp_id):
        goal_msg = NavGoal.Goal()
        goal_msg.x = waypoint[1]
        goal_msg.y = waypoint[2]
        goal_msg.theta = waypoint[2]
        goal_msg.se2 = waypoint[0]    
        goal_msg.theta_global = waypoint[3]
        goal_msg.goal_id = goal_id   
        goal_msg.wp_id = wp_id

        self.get_logger().info(f'===Goal number {goal_id}. WP number {wp_id}===') 
        self.get_logger().info(f'Local waypoint goal is: ({goal_msg.x}, {goal_msg.theta})')
        self.get_logger().info(f'Global waypoint goal is: {waypoint[0]}')
        # self.get_logger().info('Waiting for nav client...')
        self.nav_client.wait_for_server()
        # self.get_logger().info("Sending goal.")

        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.nav_get_feedback_callback)
        self.send_goal_future.add_done_callback(self.nav_goal_response_callback)
        #self.get_logger().info("Sent goal.")

    '''
    Goal Response Callback: Goal accepted/rejected?
        Inputs: future 
        Outputs: None
    '''
    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f'Planner has rejected goal for {self.name}!')
            return

        #self.get_logger().info(f'Planner has accepted goal for {self.name}')
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
        goal_id = feedback_msg.feedback.goal_id
        wp_id = feedback_msg.feedback.wp_id
        self.get_logger().info(f'Goal number {goal_id}. WP number {wp_id}\n'
            f'Remaining pose difference: x: {x_delta:.3f} y: {y_delta:.3f} theta(deg): {theta_delta*180.0/math.pi:.3f}')

    '''
    Goal Result Callback: Goal result
        Inputs: future
        Outputs: None
    '''
    def nav_get_result_callback(self, future):
        result = future.result().result
        goal_id = result.goal_id
        wp_id = result.wp_id
        self.get_logger().info(f'{self.name} Goal number {goal_id}. WP number {wp_id} has reached its goal.')

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
                # self.get_logger().info(
                #     f'Received vicon pose: ({trans.x:.3f}, {trans.y:.3f}, {y*180/math.pi:.3f})')
                self.time = self.get_clock().now()