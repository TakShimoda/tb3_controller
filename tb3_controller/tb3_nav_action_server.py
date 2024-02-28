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
import argparse, math, threading, time, yaml

'''
    TODO: GOAL POLICY: QUEUE GOALS
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

        #Parameters
        self.timer = self.get_clock().now()         #Timer for frequency of logging certain callbacks, e.g. vicon topics
        self.thread_lock = threading.Lock()         #For locking parameters in multithreading
        self.cmd_vel =  Twist()                     #Twist message to publish on cmd_vel
        self.name = name                            #Name of robot e.g. B04
        self.goal_queue = []                        #Queue of goals
        self.queue_goal = True                      #Queue goal? T/F
        self.goal_handle: ServerGoalHandle = None   #Current goal handle

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
        self.delay = config['time']['cmd_delay']
        self.x_limit = config['limits']['x']
        self.theta_limit = config['limits']['theta']

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
        self.get_logger().info(f'Received request, {request.topic}, returning pose to client: '
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

        #Initialize the action 
        self.get_logger().info(f'Executing the goal {goal_id}, waypoint number {wp_id}.' 
                f' Heading to global coordinates: {goal_se2[2]:.3f}, {goal_se2[5]:.3f}, {goal_theta_global*180.0/math.pi:.3f}')
        feedback = NavGoal.Feedback()
        result = NavGoal.Result()
        
        #First do primitive calculated motions, then finish with P-controller
        self.get_logger().info('Initiating basic motion control...')

        self.cmd_vel.linear.x = goal_x/self.delay
        self.cmd_vel.angular.z = goal_theta/self.delay

        #If velocities > limits, subtract to get it to limit
        self.cmd_vel.linear.x -= (self.cmd_vel.linear.x>self.x_limit)*(self.cmd_vel.linear.x-self.x_limit)
        self.cmd_vel.angular.z -= (self.cmd_vel._angular.z>self.theta_limit)*(self.cmd_vel._angular.z-self.theta_limit)
        self.get_logger
        self.vel_publisher.publish(self.cmd_vel)
        time.sleep(self.delay)

        #Reset to zero
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(self.cmd_vel)

        #Calculate the remaining difference between current and goal pose
        diff_x = goal_se2[2] - self.pose_x
        diff_y = goal_se2[5] - self.pose_y
        diff_theta = goal_theta_global - self.pose_theta
        self.get_logger().info(
            f'Goal {goal_id}, waypoint {wp_id}. Remaining differences after basic motion control (x, y, theta(deg)): '
            f'({diff_x:.3f}, {diff_y:.3f}, {diff_theta*180.0/math.pi:.3f})')

        self.basic_P(feedback, diff_x, diff_y, diff_theta, goal_se2[2], goal_se2[5], goal_theta_global, goal_handle)

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

        return result

    '''
    Basic P: Basic P example for linear motion
        Inputs: feedback, goal_x, goal_y, goal_theta, goal_handle
        Outputs: None
    ''' 
    def basic_P(self, feedback, diff_x, diff_y, diff_theta, goal_x, goal_y, goal_theta, goal_handle):
        
        while not (abs(diff_x) < self.diff_x and abs(diff_y) < self.diff_y and abs(diff_theta) < self.diff_theta):
            #Update poses by adding P*difference, only if difference is still above threshold
            self.cmd_vel.linear.x += self.P*diff_x*(abs(diff_x) > self.diff_x)
            self.cmd_vel.angular.z += self.P*diff_theta*(abs(diff_theta) > self.diff_theta)
            #If velocities > limits, subtract to get it to limit
            self.cmd_vel.linear.x -= (self.cmd_vel.linear.x>self.x_limit)*(self.cmd_vel.linear.x-self.x_limit)
            self.cmd_vel.angular.z -= (self.cmd_vel._angular.z>self.theta_limit)*(self.cmd_vel._angular.z-self.theta_limit)
        
            self.vel_publisher.publish(self.cmd_vel)

            #Update pose differential
            diff_x = goal_x - self.pose_x
            diff_y = goal_y - self.pose_y
            diff_theta = goal_theta - self.pose_theta

            #Send feedback
            feedback.x_delta = diff_x
            feedback.y_delta = diff_y
            feedback.theta_delta = diff_theta
            feedback.goal_id = goal_handle.request.goal_id
            feedback.wp_id = goal_handle.request.wp_id
            
            if (self.get_clock().now() - self.timer) > Duration(seconds=1.0):
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(
                    f'Current pose for goal {goal_handle.request.goal_id}, waypoint {goal_handle.request.wp_id}: ' 
                    f'{self.pose_x:.3f}, {self.pose_y:.3f}, {self.pose_theta*180.0/math.pi:.3f}\n'
                    f'Remaining pose difference {diff_x:.3f}, {diff_y:.3f}, {diff_theta*180.0/math.pi:.3f}')
                self.timer = self.get_clock().now()
        
            #Reset
            time.sleep(self.delay)
            # self.cmd_vel.linear.x = 0.0
            # self.cmd_vel.angular.z = 0.0

        #Stop robot when finished
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(self.cmd_vel)
        self.get_logger().info('PID control finished.')
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
    Vicon Pose Callback: Get the vicon pose?
        Inputs: pose(TransformStamped)
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
        
        if (self.get_clock().now() - self.timer) > Duration(seconds=1.0):
            with self.thread_lock:
                # self.get_logger().info(
                #     f'Received vicon pose: ({trans.x:.2f}, {trans.y:.2f}, {trans.z:.2f})'\
                #     f' ({r:.2f}, {p:.2f}, {y:.2f})')
                self.timer = self.get_clock().now()

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

    def process_next_goal(self):
        with self.thread_lock:
            if len(self.goal_queue)>0:
                #remove first element and execute it
                self.goal_queue.pop(0).execute()
            else:
                self.goal_handle = None

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