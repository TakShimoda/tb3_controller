#!/usr/bin/env python3
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import TransformStamped, Twist
from tb3_interfaces.action import NavGoal
from tb3_interfaces.srv import ReturnPose
import tf_transformations

#Non-ROS imports
import csv, math, threading, time
import numpy as np

'''
TODO:
    - Break the nav_execute_callback down into more basic functions to make it more readable
'''

class NavServerNode(Node):

    '''
    Constructor
        Inputs: name, config
        Outputs: None
    '''
    def __init__(self, name, config):
        super().__init__(name+'_nav_server_node')

        self.get_logger().info(f'Started navigation node for robot {name}')


        #For PID tuning
        self.csv_file_path = 'PID_tuning.csv'
        with open(self.csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time', 'Desired Value', 'Current Value'])  # Header
        self.count = 0  #stop code when count is 1

        #Parameters
        self.timer = self.get_clock().now()         #Timer for frequency of logging certain callbacks, e.g. vicon topics
        self.thread_lock = threading.Lock()         #For locking parameters in multithreading
        self.cmd_vel =  Twist()                     #Twist message to publish on cmd_vel
        self.name = name                            #Name of robot e.g. B04
        self.goal_queue = []                        #Queue of goals
        self.queue_goal = True                      #Queue goal? T/F
        self.goal_handle: ServerGoalHandle = None   #Current goal handle
        self.velocity_sign = 1.0                    #W09 has reversed polarity
        self.is_circle = lambda x, w: (abs(w) >= 0.001 and abs(x) >= 0.02)  #check if motion is circular          
        if self.name == 'W09':
            self.velocity_sign = -1.0

        #Initial pose (vicon coordinates)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0

        #Motion parameters
        self.config = config
        self.use_PID = self.config['use_PID']                        
        self.P_linear = self.config['PID_linear']['P']
        self.I_linear = self.config['PID_linear']['I']
        self.D_linear = self.config['PID_linear']['D']
        self.P_angular = self.config['PID_angular']['P']
        self.I_angular = self.config['PID_angular']['I']
        self.D_angular = self.config['PID_angular']['D']
        self.diff_x = config['diff']['x']
        self.diff_y = config['diff']['y']
        self.diff_theta = config['diff']['theta']
        self.delay = config['time']['cmd_delay']
        self.x_limit = config['limits']['x']
        self.theta_limit = config['limits']['theta']
        self.theta_limit_turn = config['limits']['theta_turn']
        self.num_points = config['line']['num_points']
        # Increase omega if waffle
        if self.name[0] == 'W':
            self.omega_scale = config['waffle'][name+'_scale_factor']
            self.get_logger().info(f'Using {name}. Adjusting omega velocities by {self.omega_scale}')
        else:
            self.omega_scale = 1.0

        # Specific cases for certain robots
        # For W01, angular speed limit is 1.82
        if self.name == 'W01':
            self.theta_limit_turn = config['W01_limits']['theta_turn'] 
        
        if self.use_PID:
            self.get_logger().info('Using PID')
        else:
            self.get_logger().info('Not using PID')

        #Communication objects
        self.nav_action_server = ActionServer(
            self, NavGoal, name+'_nav_action',
            execute_callback=self.nav_execute_callback,
            goal_callback=self.nav_goal_callback,
            cancel_callback=self.nav_cancel_callback,
            handle_accepted_callback=self.nav_handle_accepted_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        
        self.pose_server = self.create_service(ReturnPose, name+'_pose', self.send_pose)

        self.vicon_sub = self.create_subscription(
            TransformStamped,
            f'vicon/{name}/{name}',
            self.vicon_pose_callback,
            10)
        
        self.vel_publisher = self.create_publisher(
            Twist,
            self.name + "/cmd_vel",
            10)

    '''
    Send Pose: Send pose back to the client
        Inputs: request, response (ReturnPose srv)
        Outputs: response (ReturnPose srv)
    ''' 
    def send_pose(self, request, response):
        response.x = self.pose_x
        response.y = self.pose_y
        response.theta = self.pose_theta
        self.get_logger().info(f'Received request, {request.topic}, returning vicon pose to client: '
                               f'({response.x:.3f}, {response.y:.3f}, {response.theta*180.0/math.pi:.3f})')
        return response

    '''
    Execute Callback: Execute the goal
        Input: goal_handle: ServerGoalHandle
        - goals are already in the local frame, so no differential calculation is needed
        Outputs: None
    '''         
    def nav_execute_callback(self, goal_handle: ServerGoalHandle):

        # Set class attribute goal handle to current goal handle
        with self.thread_lock:
            self.goal_handle = goal_handle
        
        # Get request from client
        goal_x = goal_handle.request.x
        goal_y = goal_handle.request.y
        goal_theta = goal_handle.request.theta
        goal_se2 = goal_handle.request.se2
        goal_theta_global = goal_handle.request.theta_global
        goal_id = goal_handle.request.goal_id
        wp_id = goal_handle.request.wp_id
        x, y = goal_se2[5], -goal_se2[2]

        # Initialize the action 
        self.get_logger().info(f'Executing the goal {goal_id}, waypoint number {wp_id}.' 
                f' Heading to global vicon coordinates: {x:.3f}, {y:.3f}, {goal_theta_global*180.0/math.pi:.3f}\n.'
                f' Distance in local coordinates: x = {goal_x:.3f}, theta = {goal_theta*180.0/math.pi:.3f}')
        feedback = NavGoal.Feedback()
        result = NavGoal.Result()
        
        # First do primitive calculated motions, then finish with P-controller
        self.get_logger().info('Initiating basic motion control...')

        self.cmd_vel.linear.x = goal_x/self.delay
        
        #If circle, define angular velocity by w=v/r.
        if self.is_circle:
            goal_radius = goal_x/goal_theta
            self.cmd_vel.angular.z = self.cmd_vel.linear.x/goal_radius
        else:
            self.cmd_vel.angular.z = goal_theta/self.delay

        # If velocities > limits, subtract to set it to the limits. 
            # For linear, it's 0.18. 
            # Angle has two limits: 0.18 for linear+angular motion, and 2.6 which is the absolute limit
            # Don't limit angular to 0.18 if it's just turning
        self.cmd_vel.linear.x -= (self.cmd_vel.linear.x>self.x_limit)*(self.cmd_vel.linear.x-self.x_limit)
        self.cmd_vel.angular.z -= \
            (self.cmd_vel.angular.z>self.theta_limit_turn)*(self.cmd_vel.angular.z-self.theta_limit_turn)
        
        #if abs(self.cmd_vel.linear.x) >= 0.02: #if circular
        if self.is_circle:
            self.cmd_vel.angular.z -= (self.cmd_vel.angular.z>self.theta_limit)*(self.cmd_vel.angular.z-self.theta_limit)
            #correct for when v=w=0.18 (circ of radius 1), but we want smaller radius
            #make the linear velocity slower
            if goal_radius < (self.cmd_vel.linear.x/self.cmd_vel.angular.z):
                self.cmd_vel.linear.x = self.cmd_vel.angular.z*goal_radius
                self.get_logger().info(f'Correcting velocity to {self.cmd_vel.linear.x} for circular motion')
        # If straight line, do the straight line motion with PID correction for angles
        if (abs(self.cmd_vel.angular.z) <= 0.001 and abs(self.cmd_vel.linear.x) >= 0.02):
            with self.thread_lock:
                diff_theta = goal_theta_global - self.pose_theta 
            self.PID_straight(goal_x, diff_theta, goal_theta_global)
        # If turning/circular motion
        else:
            # If delay isn't long enough given the velocity, extend it
            if abs(self.cmd_vel.linear.x) >= 0.02: #if straight line/circle
                if self.cmd_vel.linear.x*self.delay < goal_x:
                    delay = goal_x/self.cmd_vel.linear.x
                else:
                    delay = self.delay 
            else: #angular turn
                if self.cmd_vel.angular.z*self.delay < goal_theta:
                    delay = goal_theta/self.cmd_vel.angular.z
                else:
                    delay = self.delay
            
            self.get_logger().info(
                f'Moving with speeds v = {self.cmd_vel.linear.x:.3f}, w = {self.cmd_vel.angular.z:.3f} for {delay:.3f} seconds')
            
            if self.name[0] == 'W':
                with self.thread_lock:
                    delay = self.scale_omega(delay)
            self.vel_publisher.publish(self.cmd_vel)

            time.sleep(delay)

        #Reset to zero
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(self.cmd_vel)

        #Calculate the remaining difference between current and goal pose
        #Convert the goal se(2) from cartesian to vicon (i.e. x^v = y^p, y^v = -x^p)
        with self.thread_lock:
            diff_x = goal_se2[5] - self.pose_x
            diff_y = -goal_se2[2] - self.pose_y
            diff_theta = goal_theta_global - self.pose_theta
        self.get_logger().info(
            f'Goal {goal_id}, waypoint {wp_id}. Remaining differences after basic motion control (x, y, theta(deg)): '
            f'({diff_x:.3f}, {diff_y:.3f}, {diff_theta*180.0/math.pi:.3f})')

        #convert goals to local coordinates
        diff_local = self.transform_vicon_to_robot(diff_x, diff_y, diff_theta)
        diff_x_local, diff_y_local = diff_local[0, 2], diff_local[1, 2]
        if self.use_PID:
            result.x_final, result.y_final, result.theta_final = \
                self.PID(feedback, diff_x_local, diff_y_local, diff_theta, x, y, goal_theta_global, goal_handle)
        else:
            with self.thread_lock:
                result.x_final = self.pose_x
                result.y_final = self.pose_y
                result.theta_final = self.pose_theta
            self.get_logger().info(f'No PID')
                
        #Set final goal state
        goal_handle.succeed()

        #Set and return the result
        # with self.thread_lock:
        #     result.x_final = self.pose_x
        #     result.y_final = self.pose_y
        #     result.theta_final = self.pose_theta
        result.goal_id = goal_id
        result.wp_id = wp_id

        #If we're queuing, execute the next goal inside the queue
        if self.queue_goal: 
            self.process_next_goal()
        self.get_logger().info(
            f'Reached final state for goal {goal_id} waypoint {wp_id}: ' 
            f'{result.x_final:.3f}, {result.y_final:.3f}, {result.theta_final*180.0/math.pi:.3f}')
        #set to zero
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0

        if self.count == 1:
            self.get_logger().info('Destroying node...')
            self.destroy_node()

        return result

    '''
    PID: PID example for linear motion
        Inputs: feedback, local x diff, local y diff, theta diff, goal_x, goal_y, goal_theta, goal_handle
        Outputs: x_final, y_final, theta_final
    ''' 
    def PID(self, feedback, diff_x_local, diff_y_local, diff_theta, goal_x, goal_y, goal_theta, goal_handle):
        
        #Initialize prior values in PID control
        I_prior_lin = 0.0
        diff_x_local_prior = 0.0
        I_prior_ang = 0.0
        diff_theta_prior = 0.0

        while not (abs(diff_x_local) < self.diff_x and abs(diff_y_local) < self.diff_y and abs(diff_theta) < self.diff_theta):
            #Update poses by adding P*difference, only if difference is still above threshold
            with self.thread_lock:

                #Integral/Derivatives
                I_lin = I_prior_lin + diff_x_local*self.delay
                D_lin = (diff_x_local-diff_x_local_prior)/self.delay
                I_ang = I_prior_ang + diff_theta*self.delay
                D_ang = (diff_theta-diff_theta_prior)/self.delay

                #Update
                self.cmd_vel.linear.x = (self.P_linear*diff_x_local/self.delay\
                    + (self.I_linear*I_lin) + (self.D_linear*D_lin))*(abs(diff_x_local) > self.diff_x)
                self.cmd_vel.angular.z = (self.P_angular*diff_theta/self.delay\
                    + (self.I_angular*I_ang) + (self.D_angular*D_ang))*(abs(diff_theta) > self.diff_theta)

                #If speeds > limits, subtract to get it to limit
                if self.cmd_vel.linear.x > self.x_limit:
                    self.cmd_vel.linear.x -= (self.cmd_vel.linear.x-self.x_limit)
                if self.cmd_vel.linear.x < -self.x_limit:
                    self.cmd_vel.linear.x -= (self.cmd_vel.linear.x+self.x_limit)

                if self.cmd_vel.angular.z > self.theta_limit_turn:
                    self.cmd_vel.angular.z -= (self.cmd_vel.angular.z-self.theta_limit_turn) 
                if self.cmd_vel.angular.z < -self.theta_limit_turn:
                    self.cmd_vel.angular.z -= (self.cmd_vel.angular.z+self.theta_limit_turn)
                
                #If moving forward+turn (i.e. circle), limit angular velocity to 0.18 rad/s
                if abs(self.cmd_vel.linear.x) >= 0.02:
                    self.cmd_vel.angular.z -= \
                        (self.cmd_vel.angular.z>self.theta_limit)*(self.cmd_vel.angular.z-self.theta_limit)

                #If 1) rotating, 2) diff_theta < 3 degrees, 3) omega < 0.01, increase omega
                if abs(self.cmd_vel.linear.x) < 0.005 and abs(self.cmd_vel.angular.z) < 0.01\
                      and abs(diff_theta*180.0/math.pi) < 5.0:
                    self.cmd_vel.angular.z = self.cmd_vel.angular.z*abs(diff_theta*180.0/math.pi)*20.0
                    if (self.get_clock().now() - self.timer) > Duration(seconds=0.2): 
                        self.get_logger().info(f'=====ALMOST COMPLETE: diff_theta={diff_theta:.3f}, speed={self.cmd_vel.angular.z}=====')
                
                #Scale omega once more; most it can be is 1.11 rad/s, so it won't go over limit
                self.cmd_vel.angular.z = self.cmd_vel.angular.z*self.omega_scale
                #Correct sign for W09
                self.cmd_vel.linear.x = self.cmd_vel.linear.x * self.velocity_sign
                self.cmd_vel.angular.z = self.cmd_vel.angular.z * self.velocity_sign
                self.vel_publisher.publish(self.cmd_vel)
                time.sleep(self.delay)

                #Update pose differential in global coordinates. 
                diff_x = goal_x - self.pose_x
                diff_y = goal_y - self.pose_y
                diff_theta = goal_theta - self.pose_theta

            #Use minimum error for angle to account for discontinuities at 180
            if diff_theta > math.pi:
                diff_theta -= 2*math.pi 
            if diff_theta < -math.pi:
                diff_theta += 2*math.pi

            #Set current values to prior values of next loop in PID
            I_prior_lin = I_lin
            diff_x_local_prior = diff_x_local
            I_prior_ang = I_ang
            diff_theta_prior = diff_theta

            #Convert pose differential to local coordinates
            diff_local = self.transform_vicon_to_robot(goal_x, goal_y, goal_theta)
            diff_x_local = diff_local[0, 2]
            diff_y_local = diff_local[1, 2]

            #Send feedback
            feedback.x_delta = diff_x
            feedback.y_delta = diff_y
            feedback.theta_delta = diff_theta
            feedback.goal_id = goal_handle.request.goal_id
            feedback.wp_id = goal_handle.request.wp_id
            
            #Log the current state every 0.2 seconds
            if (self.get_clock().now() - self.timer) > Duration(seconds=0.2):
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(
                    f'Current pose for goal {goal_handle.request.goal_id}, waypoint {goal_handle.request.wp_id}: ' 
                    f'{self.pose_x:.3f}, {self.pose_y:.3f}, {self.pose_theta*180.0/math.pi:.3f}\n'
                    f'Remaining pose difference (global) {diff_x:.5f}, {diff_y:.5f}, {diff_theta*180.0/math.pi:.5f}\n'
                    f'Remaining pose difference (local) {diff_x_local:.5f}, {diff_y_local:.5f}\n'
                    f'Current control velocities: x = {self.cmd_vel.linear.x:.5f}, w = {self.cmd_vel.angular.z:.5f}')
                self.timer = self.get_clock().now()
            
            #Write the differential to a csv, for PID tuning purposes
            # with open(self.csv_file_path, mode='a', newline='') as file:
            #     writer = csv.writer(file)
            #     writer.writerow([time.time(), 0, diff_theta])
        
        #Set the final poses upon stopping PID
        with self.thread_lock:
            x_final = self.pose_x
            y_final = self.pose_y
            theta_final = self.pose_theta

        #Stop robot when finished
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(self.cmd_vel)
        self.get_logger().info('PID control finished.')
        #self.count += 1
        return x_final, y_final, theta_final

    '''
    PID Straight: PID for correcting angles for straight line motion
        Inputs: feedback, goal_x, goal_y, goal_theta, goal_handle
        Outputs: None
    ''' 
    def PID_straight(self, goal_x, diff_theta, setpoint_theta):

        # Scaling factor, as movement shouldn't diverge from being straight
        scale=0.5

        #Lower delay to not overshoot the goal
        delay = self.delay*0.8

        # Initialize prior values in PID control
        I_prior_ang = 0.0
        diff_theta_prior = 0.0

        # Calculate number of points given the goal distance, velocity, and delay
        num_points = int(goal_x//(self.cmd_vel.linear.x*self.delay))
        remainder = goal_x/(self.cmd_vel.linear.x*self.delay) - num_points


        for _ in range(num_points):
            with self.thread_lock:
                # Integral/Derivatives
                I_ang = I_prior_ang + diff_theta*self.delay
                D_ang = (diff_theta-diff_theta_prior)/self.delay

                self.cmd_vel.angular.z = (self.P_angular*diff_theta/self.delay\
                + (self.I_angular*I_ang) + (self.D_angular*D_ang))*(abs(diff_theta) > self.diff_theta)

                if self.cmd_vel.angular.z > self.theta_limit:
                    self.cmd_vel.angular.z -= (self.cmd_vel.angular.z-self.theta_limit) 
                if self.cmd_vel.angular.z < -self.theta_limit:
                    self.cmd_vel.angular.z -= (self.cmd_vel.angular.z+self.theta_limit)

                #First scale down omega, then, scale it back up if it's a waffle. Don't change delay here as it's not needed
                self.cmd_vel.angular.z*=scale
                self.cmd_vel.angular.z = self.cmd_vel.angular.z*self.omega_scale
                #Correct sign for W09
                self.cmd_vel.linear.x = self.cmd_vel.linear.x * self.velocity_sign
                self.cmd_vel.angular.z = self.cmd_vel.angular.z * self.velocity_sign
                self.vel_publisher.publish(self.cmd_vel)
                time.sleep(delay)

                diff_theta = setpoint_theta - self.pose_theta

            # Use minimum error for angle to account for discontinuities at 180
            if diff_theta > math.pi:
                diff_theta -= 2*math.pi 
            if diff_theta < -math.pi:
                diff_theta += 2*math.pi

            # Set current values to prior values of next loop in PID
            I_prior_ang = I_ang
            diff_theta_prior = diff_theta

            if (self.get_clock().now() - self.timer) > Duration(seconds=0.2):
                self.get_logger().info(
                    f'Current pose: {self.pose_x:.3f}, {self.pose_y:.3f}, {self.pose_theta*180.0/math.pi:.3f}\n'
                    f'Theta offset: {diff_theta*180.0/math.pi:.5f}\n'
                    f'Current control velocities: x = {self.cmd_vel.linear.x:.5f}, w = {self.cmd_vel.angular.z:.5f}')
                self.timer = self.get_clock().now()

            #Zero the velocity
            with self.thread_lock:
                self.cmd_vel.angular.z = 0.0
                self.vel_publisher.publish(self.cmd_vel)
            
            #Write the differential to a csv, for PID tuning purposes
            # with open(self.csv_file_path, mode='a', newline='') as file:
            #     writer = csv.writer(file)
            #     writer.writerow([time.time(), setpoint_theta*180.0/math.pi, self.pose_theta*180.0/math.pi])
            
        #Stop robot when finished
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(self.cmd_vel)
        self.get_logger().info('PID straight line control finished.')
        # with self.thread_lock:
        #     self.count += 1
        return

    '''
    Goal Callback: Accept/reject the goal?
        Inputs: goal_handle: ServerGoalHandle
        Outputs: None
    ''' 
    def nav_goal_callback(self, goal_handle: ServerGoalHandle):
        #POLICY: queue goals (not implemented here)
        goal_id = goal_handle.goal_id
        wp_id = goal_handle.wp_id
        # self.get_logger().info(f'Received goal {goal_id} waypoint {wp_id}.')
        # self.get_logger().info(f'Accepting goal {goal_id} waypoint {wp_id}.')
        return GoalResponse.ACCEPT


    '''
    Handle Accept Callback: How to deal with accepted goal?
        - If we are queuing goals, then do so. If not, simply execute accepted goals as they come.
        Inputs: goal_handle: ServerGoalHandle
        Outputs: None
    '''  
    def nav_handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        if self.queue_goal:
            with self.thread_lock:
                if self.goal_handle is not None:
                    self.goal_queue.append(goal_handle)
                    self.goal_queue.sort(key=lambda goal:(goal.request.goal_id, goal.request.wp_id))
                    #print(f'Adding goal {goal_handle.request.goal_id}, waypoint {goal_handle.request.wp_id} to queue')
                else:
                    self.get_logger().info(f'Proceeding to execute accepted goal' 
                                        f' {goal_handle.request.goal_id}, waypoint {goal_handle.request.wp_id}')
                    goal_handle.execute()
        else:
            goal_handle.execute()

    '''
    Process Next Goal: Process next goal in the queue
        - Check if queue is empty; if so, set goal handle attribute to none. Else, pop and execute the first goal in queue
        Inputs: None
        Outputs: None
    ''' 
    def process_next_goal(self):
        with self.thread_lock:
            if len(self.goal_queue)>0:
                #remove first element and execute it
                self.goal_queue.pop(0).execute()
            else:
                self.goal_handle = None

    '''
    Vicon Pose Callback: Get the vicon pose?
        Inputs: pose(TransformStamped) - in vicon coordinates (x-forward, y-left)
        Outputs: None
    ''' 
    def vicon_pose_callback(self, pose):

        trans = pose.transform.translation
        quat = pose.transform.rotation
        r, p, y = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        with self.thread_lock:
            #Correct for gimbal lock around 180/-180 degrees
                # if magnitudes of last/current reading are similar, their magnitudes > 170, and they have opposite signs
            # if (0.9 < abs(self.pose_theta/y) < 1.10) \
            #     and abs(self.pose_theta) > 170 and abs(y) > 170 and self.pose_theta/y < 0:
            # if abs(self.pose_theta) > 170 and abs(y) > 170 and self.pose_theta/y < 0 and y < -90:
            # if y < 0:
            #     #self.get_logger().warn('GIMBAL LOCK: CORRECTING.....')
            #     y += 2*math.pi

            self.pose_x = trans.x
            self.pose_y = trans.y
            self.pose_theta = y
        
        if (self.get_clock().now() - self.timer) > Duration(seconds=1.0):
            # with self.thread_lock:
            #     self.get_logger().info(
            #         f'Received vicon pose: (x: {trans.x:.3f}, y: {trans.y:.3f}, z:{trans.z:.3f}\n'
            #         f'r: {r*180/math.pi:.3f}, p: {p*180/math.pi:.3f}, y: {y*180/math.pi:.3f})')
                self.timer = self.get_clock().now()

    '''
    Transform from Vicon to Robot coordinates. For the purposes of PID correction
        Inputs: (x, y, theta): position of goal (or goal differential) in global coordinates
        Outputs: pose_goal_local: SE(2) in local coordinates
    '''
    def transform_vicon_to_robot(self, x, y, theta):
        pose_goal_global = np.array([[np.cos(theta), -np.sin(theta), x],
                                [np.sin(theta), np.cos(theta), y],
                                [0, 0, 1]])
        with self.thread_lock:
            pose_robot_global = np.array([[np.cos(self.pose_theta), -np.sin(self.pose_theta), self.pose_x],
                                [np.sin(self.pose_theta), np.cos(self.pose_theta), self.pose_y],
                                [0, 0, 1]])
            pose_goal_local = np.linalg.inv(pose_robot_global)@pose_goal_global
        
        return pose_goal_local

    '''
    Scale omega. For the purposes of waffle bots to increase their angular velocity
        Input: delay
        Output: delay
    '''    
    def scale_omega(self, delay):
        if abs(self.cmd_vel.linear.x) > 0.02: #if straight/circle
            if self.cmd_vel.angular.z*self.omega_scale >= self.theta_limit:
                self.get_logger().info(f'Increasing delay for linear/circular motion from'
                                        f'{delay:.3f} to {delay*self.omega_scale:.3f}')
                delay = delay*self.omega_scale
            else:
                self.get_logger().info(f'Increasing speed for linear/circular motion from'
                    f'{self.cmd_vel.angular.z:.3f} to {self.cmd_vel.angular.z*self.omega_scale}')
                self.cmd_vel.angular.z = self.cmd_vel.angular.z*self.omega_scale
                
        else: # if turning
            if self.cmd_vel.angular.z*self.omega_scale >= self.theta_limit_turn:
                delay = delay*self.omega_scale 
                self.get_logger().info(f'Increasing delay for turning motion from'
                                        f'{delay:.3f} to {delay*self.omega_scale:.3f}')
            else:
                self.cmd_vel.angular.z = self.cmd_vel.angular.z*self.omega_scale
                self.get_logger().info(f'Increasing speed for turning motion from'
                    f'{self.cmd_vel.angular.z:.3f} to {self.cmd_vel.angular.z*self.omega_scale}')
                
        #Correct sign for W09
        self.cmd_vel.linear.x = self.cmd_vel.linear.x * self.velocity_sign
        self.cmd_vel.angular.z = self.cmd_vel.angular.z * self.velocity_sign

        return delay 
#############################################
#========EMPTY FUNCTIONS FOR NOW============#
#############################################
    '''
    Cancel Callback: How to handle canceled goal?
        Inputs: goal_handle: ServerGoalHandle
        Outputs: None
    ''' 
    def nav_cancel_callback(self, goal_handle: ServerGoalHandle):
        #For now, nothing
        pass