#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import String
from tb3_interfaces.action import NavGoal
from tb3_interfaces.srv import ReturnPose
import tf_transformations

#Non-ROS imports
import argparse, csv, math, threading, time, yaml
import numpy as np

'''
TODO:
    - Break the nav_execute_callback down into more basic functions to make it more readable
'''

class NavNode(Node):

    '''
    Constructor
        Inputs: name 
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

        #Initial pose (vicon coordinates)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0

        #Motion parameters
        self.config = config                        
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

        #Communication objects
        self.nav_action_server = ActionServer(
            self, NavGoal, 'nav_action',
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

    def send_pose(self, request, response):
        response.x = self.pose_x
        response.y = self.pose_y
        response.theta = self.pose_theta
        self.get_logger().info(f'Received request, {request.topic}, returning vicon pose to client: '
                               f'({response.x:.3f}, {response.y:.3f}, {response.theta*180.0/math.pi:.3f})')
        return response

    '''
    Execute Callback: Execute the goal
        Inputs: goal_handle: ServerGoalHandle
        - goals are already in the local frame, so no differential calculation is needed
        Outputs: None
    '''         
    def nav_execute_callback(self, goal_handle: ServerGoalHandle):

        #Set class attribute goal handle to current goal handle
        with self.thread_lock:
            self.goal_handle = goal_handle
        
        #Get request from client
        goal_x = goal_handle.request.x
        goal_y = goal_handle.request.y
        goal_theta = goal_handle.request.theta
        goal_se2 = goal_handle.request.se2
        goal_theta_global = goal_handle.request.theta_global
        goal_id = goal_handle.request.goal_id
        wp_id = goal_handle.request.wp_id
        x, y = goal_se2[5], -goal_se2[2]

        #Initialize the action 
        self.get_logger().info(f'Executing the goal {goal_id}, waypoint number {wp_id}.' 
                f' Heading to global vicon coordinates: {x:.3f}, {y:.3f}, {goal_theta_global*180.0/math.pi:.3f}\n.'
                f' Distance in local coordinates: x = {goal_x:.3f}, theta = {goal_theta*180.0/math.pi:.3f}')
        feedback = NavGoal.Feedback()
        result = NavGoal.Result()
        
        #First do primitive calculated motions, then finish with P-controller
        self.get_logger().info('Initiating basic motion control...')

        self.cmd_vel.linear.x = goal_x/self.delay
        self.cmd_vel.angular.z = goal_theta/self.delay

        # If velocities > limits, subtract to set it to the limits. 
            #For linear, it's 0.18. 
            #Angle has two limits: 0.18 for linear+angular motion, and 2.8 which is the absolute limit
            # Don't limit angular (0.18) if it's just turning
        self.cmd_vel.linear.x -= (self.cmd_vel.linear.x>self.x_limit)*(self.cmd_vel.linear.x-self.x_limit)
        self.cmd_vel.angular.z -= (self.cmd_vel.angular.z>self.theta_limit_turn)*(self.cmd_vel.angular.z-self.theta_limit_turn)
        if abs(self.cmd_vel.linear.x) >= 0.02:
            self.cmd_vel.angular.z -= (self.cmd_vel.angular.z>self.theta_limit)*(self.cmd_vel.angular.z-self.theta_limit)

        #If delay isn't long enough given the velocity, extend it
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
        self.vel_publisher.publish(self.cmd_vel)
        #add a little bit more to delay
        time.sleep(delay+(delay/20))

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
        self.PID(feedback, diff_x_local, diff_y_local, diff_theta, x, y, goal_theta_global, goal_handle)

        #Set final goal state
        goal_handle.succeed()

        #Set and return the result
        result.x_final = self.pose_x
        result.y_final = self.pose_y
        result.theta_final = self.pose_theta
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
    Basic P: Basic P example for linear motion
        Inputs: feedback, goal_x, goal_y, goal_theta, goal_handle
        Outputs: None
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
                self.cmd_vel.linear.x = self.P_linear*diff_x_local*(abs(diff_x_local) > self.diff_x)/self.delay\
                    + (self.I_linear*I_lin) + (self.D_linear*D_lin)
                self.cmd_vel.angular.z = self.P_angular*diff_theta*(abs(diff_theta) > self.diff_theta)/self.delay\
                    + (self.I_angular*I_ang) + (self.D_angular*D_ang)

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
                    
                #If below minimum, change velocities so they are at the minimum

                self.vel_publisher.publish(self.cmd_vel)

            #Update pose differential in global coordinates
            diff_x = goal_x - self.pose_x
            diff_y = goal_y - self.pose_y
            diff_theta = goal_theta - self.pose_theta

            #Set current values to prior values of next loop in PID
            I_prior_lin = I_lin
            diff_x_local_prior = diff_x_local

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
            
            if (self.get_clock().now() - self.timer) > Duration(seconds=0.2):
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(
                    f'Current pose for goal {goal_handle.request.goal_id}, waypoint {goal_handle.request.wp_id}: ' 
                    f'{self.pose_x:.3f}, {self.pose_y:.3f}, {self.pose_theta*180.0/math.pi:.3f}\n'
                    f'Remaining pose difference (global) {diff_x:.5f}, {diff_y:.5f}, {diff_theta*180.0/math.pi:.5f}\n'
                    f'Remaining pose difference (local) {diff_x_local:.5f}, {diff_y_local:.5f}\n'
                    f'Current control velocities: x = {self.cmd_vel.linear.x:.5f}, w = {self.cmd_vel.angular.z:.5f}')
                self.timer = self.get_clock().now()
            
            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([time.time(), 0, diff_theta])
        
            #Reset
            time.sleep(self.delay)
            # self.cmd_vel.linear.x = 0.0
            # self.cmd_vel.angular.z = 0.0

        #Stop robot when finished
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(self.cmd_vel)
        self.get_logger().info('PID control finished.')
        #self.count += 1
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
            if y < 0:
                #self.get_logger().warn('GIMBAL LOCK: CORRECTING.....')
                y += 2*math.pi

            self.pose_x = trans.x
            self.pose_y = trans.y
            self.pose_theta = y
        
        if (self.get_clock().now() - self.timer) > Duration(seconds=1.0):
            with self.thread_lock:
                # self.get_logger().info(
                #     f'Received vicon pose: ({trans.x:.3f}, {trans.y:.3f}, {y*180/math.pi:.3f})')
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