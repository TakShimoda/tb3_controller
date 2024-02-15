#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from tb3_interfaces.action import ContactComputer
from tb3_interfaces.msg import RobotStatus

#Non-ROS imports
import argparse

class ComputerNode(Node):

    def __init__(self, num_robots):
        super().__init__('computer_node')
        self.action_server = ActionServer(
            self,
            ContactComputer,
            'group_trigger',
            self.execute_callback)
        self.robot_ids = set()                  #Unique set of all robots that contact
        self.num_robots = int(num_robots)       #Number of expected robots
        self.latest_time = 0                    #Time of latest robot to contact computer

        self.get_logger().info(f'Computer node started. Expecting {num_robots} robots.')

        # Publisher to broadcast success message to all clients
        self.success_publisher: rclpy.Publisher = self.create_publisher(
            RobotStatus, 'group_trigger_success', 10)

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal from {goal_handle.request.robot} at time: {goal_handle.request.timestamp.sec}')
        self.latest_time = goal_handle.request.timestamp
        self.robot_ids.add(goal_handle.request.robot)

        # Once all clients have sent a goal, respond to all
        goal_handle.succeed()
        result = ContactComputer.Result()
        result.success = (len(self.robot_ids) >= self.num_robots)
        result.reached_number = len(self.robot_ids)

        robot_status =  RobotStatus()
        robot_status.num_robots = len(self.robot_ids)
        robot_status.timestamp =  self.latest_time
        
        if len(self.robot_ids) == self.num_robots:
            robot_status.total_reached = True
            self.get_logger().info(f'All {self.num_robots} robots have contacted computer. Latest time:  {str(self.latest_time.sec)}')
            self.robot_ids.clear()  # Reset for the next round
        else:
             robot_status.total_reached = False
             self.get_logger().info(f'{str(len(self.robot_ids)/self.num_robots)} robots have contacted computer. Latest time:  {str(self.latest_time.sec)}')
        self.success_publisher.publish(robot_status)
        

        return result

def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--num_robots", help="Number of robots to contact", required=False, default=2)
    args = parser.parse_args()

    rclpy.init()
    node = ComputerNode(args.num_robots) 
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
