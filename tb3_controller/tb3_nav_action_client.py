#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient

from rclpy.node import Node
from rclpy.clock import ROSClock, ClockType
from rclpy.time  import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time as TimeStamp
from tb3_interfaces.action import NavGoal
from tb3_interfaces.srv import ReturnPose

#Non-ROS imports
import argparse, math, time, yaml
import numpy as np

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
    def __init__(self, name, client_config):
        super().__init__(name + '_nav_client_node')

        #Navigation client
        self.client = ActionClient(self, NavGoal, 'nav_action')
        self.get_logger().info(f'{name} nav client has initialized.')

        #Service client for getting one-time pose
        self.pose_client = self.create_client(ReturnPose, name+'_pose')

        #Parameters
        self.name = name
        self.client_config = client_config
        self.type = self.client_config['type']                                  #circular or square
        
    '''
    Create local goal: create local goal once, to repeatedly right-multiply into initial robot-pose
        Inputs: 
        - type: (circ/linear/angular)
        - dist_lin: total linear distance (m)
        - dist_theta: total angular distance (rad)
        Outputs: goal in local coordinates coordinates
    '''
    def create_local_goal(self, type, dist_lin, dist_theta):
        if type == 'circular':
            #use polar coordinates
            radius = dist_lin/dist_theta
            theta_loc = dist_theta
            print(f'The radius is: {radius}\n')
            print(f'Theta is: {theta_loc*180.0/math.pi:.3f}\n')
            x_loc = radius*np.cos(theta_loc) - radius    
            y_loc = radius*np.sin(theta_loc)
        elif type == 'linear':
            x_loc = 0.0
            y_loc = dist_lin
            theta_loc = 0.0
        else: #angular
            x_loc = 0.0
            y_loc = 0.0
            theta_loc = dist_theta

        goal_local = np.array([[np.cos(theta_loc), -np.sin(theta_loc), x_loc], 
                                [np.sin(theta_loc), np.cos(theta_loc), y_loc], 
                                [0, 0, 1]])

        np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

        return goal_local   

    '''
    Create waypoints: create equidistant waypoints in global coordinates, each as goals for server to complete
        - Moreover, this would take a larger goal (e.g. quarter circle) and break it down to smaller goals
        - Also, the global goal returned on one iteration can serve as the pose in the next iteration 
            to easily make consecutive equidistant waypoints
        - Also returns angle increments as it's hard to calculate RPY off of SO(2)
        Inputs: 
            - type: circular, linear, or other(i.e. angular)
            - total_linear: total linear distance
            - total_theta: total angular distance
            - num_points: number of points
            - (x, y, theta): current robot pose in Cartesian global coordinates 
        Outputs: waypoints [(SE(2) as np.array.flatten(), linear increment, angle increment, global theta)]
    '''
    def create_waypoints(self, type, total_linear, total_theta, num_points, x, y, theta):
        waypoints = []
        dist_lin = total_linear/num_points
        dist_theta = total_theta/num_points 
        #Create initial SE(2)
        SE_2 = np.array([[np.cos(theta), -np.sin(theta), x], 
                    [np.sin(theta), np.cos(theta), y],
                    [0.0, 0.0, 1.0]], dtype=np.float32)
        goal_global = SE_2

        #Create local goal to repeatedly right-multiply to robot pose to accumulate waypoints
        goal_local = self.create_local_goal(type, dist_lin, dist_theta)
        np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
        print(f"====Initial Robot Pose======\n{goal_global}\n")
        print(f'====local waypoint goal for type {type}, repeated {num_points} times======\n{goal_local}\n')

        for i in range(num_points):
            #calculate global points in Cartesian
            goal_global = (goal_global@goal_local).astype(dtype=np.float32)
            #add the increment to the global theta
            theta += dist_theta
            theta -= (2*math.pi*(theta>math.pi))
            waypoints.append((goal_global.flatten(), dist_lin, dist_theta, theta))

        return waypoints

    '''
    Create all goals: create all subgoals and their respective waypoints
        - get the robot pose (with a service asking for the pose one time)
        - iterate through all the sub-goals (e.g. quarter circle), get waypoints for each, and execute each subgoal
        Inputs: None 
        Outputs: goals queue: [waypoints]
    '''
    def create_all_goals(self, type_):
        #Initialize parameters
        if type_ == 'circular':
            dist_lin =  self.client_config['circ']['rad']*self.client_config['circ']['angle']*math.pi/180.0
            dist_theta = self.client_config['circ']['angle']*math.pi/180.0
            num_goals = int(2*math.pi//dist_theta) #number of subarcs to take
            num_waypoints = self.client_config['circ']['num_points']
        elif type_ == 'square':
            dist_lin = self.client_config['square']['dist']
            dist_theta = 0.0
            num_goals = 4
            num_waypoints = self.client_config['square']['num_points']
        else: #angular
            #Turn parameters
            dist_lin = 0.0
            dist_theta = self.client_config['turn']['angle']*math.pi/180.0
            num_goals = int(2*math.pi//dist_theta)
            num_waypoints = self.client_config['turn']['num_points']
            
        self.get_logger().info(f'There are {num_goals} goals and {num_waypoints} waypoints for each.')
        self.get_logger().info("Waiting to get current pose to start planning waypoints ...")
        self.total_waypoints_goal = num_goals*num_waypoints
        current_pose = self.get_pose()

        #We get poses in vicon coordinates. Convert to Cartesian coordinates (theta remains the same)
        x = -current_pose.y
        y = current_pose.x
        theta = current_pose.theta

        #Iterate over all subgoals to create all goals
        goals_queue = []
        for i in range(num_goals):
            self.get_logger().info(f'=====Making goal number {i}.' 
                    f' Current robot pose (Cartesian): x= {x:.3f}, y={y:.3f}, theta={theta*180.0/math.pi:.3f}.=====')

            if type_ == 'square':
                #straight
                waypoints = self.create_waypoints('linear', dist_lin, dist_theta, num_waypoints, x, y, theta)
                goals_queue.append(waypoints)
                x, y = waypoints[-1][0][2], waypoints[-1][0][5]
                theta += dist_theta
                theta -= (2*math.pi*(theta>math.pi))
                #Turn
                waypoints = self.create_waypoints('angular', 0.0, math.pi/2, 1, x, y, theta)
                goals_queue.append(waypoints)
                x, y = waypoints[-1][0][2], waypoints[-1][0][5]
                theta += math.pi/(2*num_waypoints)
                theta -= (2*math.pi*(theta>math.pi))
            else:
                waypoints = self.create_waypoints(type_, dist_lin, dist_theta, num_waypoints, x, y, theta)
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
        current_goal = self.send_all_nav_goals(self.type, current_goal, 2)
       # self.get_logger().info(f'Completed all motion. {self.name} finished.')

    '''
    Send all navigation goals
        Inputs: 
            - type: circular, linear, or angular 
            - current_goal: the current goal ID in order to stack goal ID of different motions
        Outputs: None
    '''
    def send_all_nav_goals(self, type_, current_goal, repeat=1):
        goals_queue = self.create_all_goals(type_)
        #repeat n times
        goals_queue = goals_queue*repeat
        self.get_logger().info(f'Repeating trajectory {repeat} times.')
        for goal_id, goals in enumerate(goals_queue):
            for wp_id, waypoints in enumerate(goals):
                self.send_nav_goal(waypoints, goal_id+current_goal, wp_id)
                #sleep to prevent first two goals executing at once
                time.sleep(0.05)
            self.get_logger().info(f'Finished sending goal {goal_id}')
        self.get_logger().info(f'Finished sending all goals for {type_} motion.')
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
        self.client.wait_for_server()
        # self.get_logger().info("Sending goal.")

        self.send_goal_future = self.client.send_goal_async(
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
    action_client.motion_planner()
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


