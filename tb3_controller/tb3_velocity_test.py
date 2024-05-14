#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

#Test velocities on turtlebot

class TB3(Node):
    def __init__(self, name):
        super().__init__(name+'test')

        self.cmd_vel =  Twist()                     #Twist message to publish on cmd_vel
        self.name = name                            #Name of robot e.g. B04
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = 1.82

        self.vel_publisher = self.create_publisher(
            Twist,
            self.name + "/cmd_vel",
            10)
        self.get_logger().info('Initializing...')
        self.pub_vel()
    def pub_vel(self):
        self.vel_publisher.publish(self.cmd_vel)
        time.sleep(3.45)
        self.cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(self.cmd_vel)
        self.get_logger().info('Done')

def main():
    rclpy.init()
    test_node = TB3('W01')
    rclpy.spin(test_node)
    rclpy.shutdown()

if __name__=='__main__':
    main()