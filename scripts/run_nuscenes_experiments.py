#!/usr/bin/env python3

import json
from pathlib import Path

import rclpy
from rclpy.node import Node

import rosbag2_py
import nuscenes.utils.splits as nuscenes_splits
    
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

        # Create ROS objects
        self.reader = rosbag2_py.SequentialReader()
    
        # Create member variables
        self.results_dict = dict()

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

            self.get_logger().info("Loading tracker experiment configuration: %s" % (exp))

            # Initialize results
            self.results_dict = dict()
            self.add_result_metadata()

            # Iterate through scene, messages

            # Write to results dict

            # Write results to json file
            with open(self.results_dir / "_".join([exp, "results.json"]), "w") as outfile:
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