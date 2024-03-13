#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import ROSClock, ClockType
from rclpy.time  import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time as TimeStamp
from tb3_interfaces.action import ContactComputer, NavGoal
from tb3_interfaces.msg import RobotStatus

#Non-ROS imports
import argparse, time, yaml

class TB3Node(Node):

    '''
    Constructor
        Inputs: 
            - name(string): the name of the robot instantiated. e.g. B04
            - trajectory(string): the trajectory to move e.g. 'circle'
        Outputs: None
    '''

    def __init__(self, name, motion_config):
        super().__init__(name + 'node')

        #Class members
        self.msg_logged = False                 #Already logged result from server? Only for purpose of last robot so it logs server result before subscription result
        self.log_msg = ''                       #Store log message in case it's the last robot.
        self.last_time = TimeStamp              #Time for last robot
        self.name = name                        #Name of robot

        #Movement parameters
        self.motion_config = motion_config      #Configuration of motion
        self.start = False                      #True/False whether to start moving the robot yet
        self.cmd_vel = Twist()                  #cmd_vel commands to send to the robot

        #Action client for initializing robot with computer
        self.client = ActionClient(self, ContactComputer, 'group_trigger')
        self.navclient = ActionClient(self, NavGoal, 'nav_action')
        #Subscriber for counting total number of robots
        self.subscription = self.create_subscription(
            RobotStatus,
            'group_trigger_success',
            self.num_robots_callback,
            10)
        self.vel_publisher = self.create_publisher(
            Twist,
            self.name + "/cmd_vel",
            10)
        
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

        self.client.wait_for_server()
        self.send_goal_future = self.client.send_goal_async(goal_msg)
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
        
        self.timer = self.create_timer(delay.nanoseconds/1e9, self.move_robot)
        self.get_logger().info(f'Timer started. Starting in {delay.nanoseconds/1e9} seconds.')


#==========MOTION FUNCTIONS==========
    '''
    Move Robot: move the robot by publishing to /<robot>/cmd_vel
        Inputs: None
        Outputs: None
    '''
    def move_robot(self):

        self.get_logger().info(f'Opened config file. {self.motion_config["type"]} motion is used.')
        self.get_logger().info(f'Initializing movements for robot {self.name}')

        #Initial parameters: z will spin on the z
            # linear.y, linear.z, angular.x, angular.y will always be zero
        self.cmd_vel.angular.z = 0.8

        #Initial spinning pattern
        for i in range(16):
            self.vel_publisher.publish(self.cmd_vel)
            self.get_logger().info(
                f'Publishing velocity commands x: {self.cmd_vel.linear.x}, theta: {self.cmd_vel.angular.z}')
            time.sleep(0.5)
        
        #Stop
        self.cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(self.cmd_vel)

        #Start trajectory motion
        self.get_logger().info(
            f'Finished spin cycle for robot {self.name}. Initiating {self.motion_config["type"]} motion.')
        if self.motion_config['type'] == 'circular': 
            self.cmd_vel.linear.x = self.motion_config['circle']['linear_x']
            self.cmd_vel.angular.z = self.motion_config['circle']['linear_x']/self.motion_config['circle']['radius']
            self.basic_motion(self.motion_config['circle']['num_cmds'], self.motion_config['circle']['delay']) 
        else:
            self.square_motion()

        self.get_logger().info(f'{self.name} has finished moving. End')
        self.timer.cancel()
    
    '''
    Square Motion: move the robot in a square by publishing to /<robot>/cmd_vel
        Inputs: None
        Outputs: None
    '''
    def square_motion(self):
        for i in range(4):
            #straight motion
            self.get_logger().info("Moving straight")
            self.cmd_vel.linear.x = self.motion_config['basic']['linear_x']
            self.basic_motion(self.motion_config['basic']['num_cmds'], self.motion_config['basic']['delay'])
            #turn motion
            self.get_logger().info("Turning.")
            self.cmd_vel.angular.z = self.motion_config['basic']['angular_z']
            self.basic_motion(self.motion_config['basic']['num_cmds'], self.motion_config['basic']['delay'])
    
    '''
    Basic Motion: most basic motion which can involve straight line or turning. Uses delays
        Inputs: None
        Outputs: None
    '''
    def basic_motion(self, num_cmd, delay):
        for i in range(num_cmd):
            self.vel_publisher.publish(self.cmd_vel)
            time.sleep(delay)
            if (i+1)%10==0:
                self.get_logger().info(f'{i+1}/{num_cmd} movements completed.')
        #Stop
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(self.cmd_vel)

#==========NAVIGATION FUNCTIONS==========
    def send_nav_goal(self):
        goal_msg = NavGoal.Goal()
        goal_msg.x = 0.0
        goal_msg.y = 0.0
        goal_msg.theta = 0.0

        self.client.wait_for_server()
        self.send_goal_future = self.client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Planner has rejected goal for {self.name}!')
            return

        self.get_logger().info(f'Planner has accepted goal for {self.name}')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.nav_get_result_callback)

    def nav_get_feedback_callback(self, feedback_msg):
        x_delta = feedback_msg.feedback.y_delta
        y_delta = feedback_msg.feedback.y_delta
        theta_delta = feedback_msg.feedback.theta_delta
        self.get_logger().info(f'Remaining pose difference: x: {x_delta} y: {y_delta} theta: {theta_delta}')

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
    action_client = TB3Node(args.name, motion_config)
    action_client.send_goal()
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
