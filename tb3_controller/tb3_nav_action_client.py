#!/usr/bin/env python3
import rclpy

from rclpy.executors import SingleThreadedExecutor
from tb3_controller.nav_client_node import NavClientNode

#Non-ROS imports
import argparse, yaml, pathlib

def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", help="robot name", required=False, default="B04")
    parser.add_argument("--config", help="path to config file", required=False, 
                        default=str(pathlib.Path(__file__).parents[3].resolve())+
                            "/src/tb3_controller/config/client_config.yaml")
    args = parser.parse_args()

    with open(args.config, 'r') as file:
        motion_config = yaml.safe_load(file)
        print(f"Client config file found at: {args.config}")

    rclpy.init()
    executor = SingleThreadedExecutor()
    action_client = NavClientNode(args.name, motion_config)
    #action_client.motion_planner()
    action_client.send_goal()
    executor.add_node(action_client)
    executor.spin()
    #rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


