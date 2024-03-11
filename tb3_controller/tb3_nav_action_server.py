#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from tb3_controller.navnode import NavNode

#Non-ROS imports
import argparse, yaml

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
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    #rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()