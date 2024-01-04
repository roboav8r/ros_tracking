#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import rosbag2_py
    
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
        self.exp_configs = self.get_parameter('val_dataset').get_parameter_value().string_value
        self.exp_configs = self.get_parameter('val_split').get_parameter_value().string_value
        self.exp_configs = self.get_parameter('mcap_dir').get_parameter_value().string_value
        self.exp_configs = self.get_parameter('results_dir').get_parameter_value().string_value

        # Create ROS objects
        self.reader = rosbag2_py.SequentialReader()
    


def main(args=None):
    rclpy.init(args=args)

    # Create experiment manager
    nusc_exp_mgr = NuscenesExpManager()

    # Run nuScenes experiments

    # Shut down the node
    nusc_exp_mgr.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()