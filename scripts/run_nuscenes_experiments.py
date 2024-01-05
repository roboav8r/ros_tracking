#!/usr/bin/env python3

import os
import json
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

import rosbag2_py
import nuscenes.utils.splits as nuscenes_splits
    
from std_srvs.srv import Empty

class NuscenesExpManager(Node):
    def __init__(self):
        super().__init__('nusc_exp_mgr_node')

        # Declare and load parameters from .yaml file
        self.declare_parameter('exp_configs', rclpy.Parameter.Type.STRING_ARRAY )
        self.declare_parameter('val_dataset', rclpy.Parameter.Type.STRING )
        self.declare_parameter('val_split', rclpy.Parameter.Type.STRING )
        self.declare_parameter('mcap_dir', rclpy.Parameter.Type.STRING )
        self.declare_parameter('results_dir', rclpy.Parameter.Type.STRING )
    
        self.exp_configs = self.get_parameter('exp_configs').get_parameter_value().string_array_value
        self.val_dataset = self.get_parameter('val_dataset').get_parameter_value().string_value
        self.val_split = self.get_parameter('val_split').get_parameter_value().string_value
        self.mcap_dir = Path.home() / self.get_parameter('mcap_dir').get_parameter_value().string_value
        self.results_dir = Path.home() / self.get_parameter('results_dir').get_parameter_value().string_value
        self.package_dir = get_package_share_directory('ros_tracking')

        # Create ROS objects
        self.reset_tracker_client = self.create_client(Empty, "reset_tracker")
        self.reconf_tracker_client = self.create_client(Empty, "reconfigure_tracker")
        self.empty_req = Empty.Request()
        self.reader = rosbag2_py.SequentialReader()
    
        # Create member variables
        self.results_dict = dict()

        # If results directory doesn't exist, create it
        if not os.path.exists(self.results_dir):
            os.mkdir(self.results_dir)

    def add_result_metadata(self):
        self.results_dict["meta"] = dict()
        self.results_dict["meta"]["use_camera"] = False
        self.results_dict["meta"]["use_lidar"] = True
        self.results_dict["meta"]["use_radar"] = False
        self.results_dict["meta"]["use_map"] = False
        self.results_dict["meta"]["use_external"] = False
        self.results_dict["results"] = dict()

    def run_experiments(self):

        # Reconfigure tracker
        for exp in self.exp_configs:

            # Load the experimental configuration for the tracker
            exp_path = os.path.join(self.package_dir,exp)
            exp_name = os.path.splitext(os.path.split(exp_path)[-1])[0]
            self.get_logger().info("Loading tracker experiment configuration: %s" % (exp_name))
            subprocess.run(["ros2","param", "load", "/tracker", os.path.join(self.package_dir,exp_path)])
            
            # Reset & reconfigure tracker
            self.future = self.reset_tracker_client.call_async(self.empty_req)
            rclpy.spin_until_future_complete(self, self.future)
            self.future = self.reconf_tracker_client.call_async(self.empty_req)
            rclpy.spin_until_future_complete(self, self.future)

            # Initialize results
            self.results_dict = dict()
            self.add_result_metadata()

            # Iterate through scene, messages

            # Write to results dict

            # Write results to json file
            with open(self.results_dir / "_".join([exp_name, "results.json"]), "w") as outfile:
                json.dump(self.results_dict, outfile, indent=2)


def main(args=None):
    rclpy.init(args=args)

    # Create experiment manager
    nusc_exp_mgr = NuscenesExpManager()

    # Run nuScenes experiments
    nusc_exp_mgr.run_experiments()

    # Shut down the node
    nusc_exp_mgr.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()