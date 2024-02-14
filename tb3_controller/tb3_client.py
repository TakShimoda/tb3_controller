#!/usr/bin/env python3
import rclpy, argparse
from rclpy.node import Node
from functools import partial
from my_robot_interfaces.srv import TB
from geometry_msgs.msg import TransformStamped
    
    
class TB3Node(Node): 
    def __init__(self, name):
        super().__init__("TB3_node")
        self.name = name
        self.get_logger().info(self.name + " node has been started.")
        self.call_computer()
        self.vicon_sub = self.create_subscription(
            TransformStamped, "/vicon/" + self.name + "/"+self.name, self.pose_callback, 10)
    
    def call_computer(self):
        client = self.create_client(TB, "TB_service")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server computer...")
        request = TB.Request()
        request.robot = self.name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_computer,robot=self.name))

    def callback_call_computer(self, future, robot):
        try:
            response = future.result()
            self.get_logger().info(
                "Robot " + robot + " called, got response: " + str(response.response))
        except Exception as e:
            self.get_logger().error("Service call failed %r" %(e,))

    def pose_callback(self, msg):
        self.get_logger().info(str(msg))
        pass

def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", help="robot name", required=False, default="B04")
    args = parser.parse_args()
    rclpy.init()
    node = TB3Node(args.name) 
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()