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

from foxglove_msgs.msg import SceneUpdate, SceneEntity
from std_srvs.srv import Empty

class NuscenesExpManager(Node):
    def __init__(self):
        super().__init__('nusc_exp_mgr_node')

        # Declare and load ROS parameters from .yaml file
        self.declare_parameter('exp_configs', rclpy.Parameter.Type.STRING_ARRAY )
        self.declare_parameter('val_dataset', rclpy.Parameter.Type.STRING )
        self.declare_parameter('val_split', rclpy.Parameter.Type.STRING )
        self.declare_parameter('mcap_dir', rclpy.Parameter.Type.STRING )
        self.declare_parameter('results_dir', rclpy.Parameter.Type.STRING )
        self.declare_parameter('det_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('lidar_detector', rclpy.Parameter.Type.STRING)
        
        self.exp_configs = self.get_parameter('exp_configs').get_parameter_value().string_array_value
        self.val_dataset = self.get_parameter('val_dataset').get_parameter_value().string_value
        self.val_split = self.get_parameter('val_split').get_parameter_value().string_value
        self.mcap_dir = Path.home() / self.get_parameter('mcap_dir').get_parameter_value().string_value
        self.results_dir = Path.home() / self.get_parameter('results_dir').get_parameter_value().string_value
        self.det_topic = self.get_parameter('det_topic').get_parameter_value().string_value
        self.lidar_detector = self.get_parameter('lidar_detector').get_parameter_value().string_value

        # Non-ros parameters
        self.package_dir = get_package_share_directory('ros_tracking')
        self.split = eval('nuscenes_splits.' + self.val_split)
        self.lidar_det_string = "-" + self.lidar_detector if self.lidar_detector else ""

        # Create ROS objects
        self.reset_tracker_client = self.create_client(Empty, "reset_tracker")
        self.reconf_tracker_client = self.create_client(Empty, "reconfigure_tracker")
        self.empty_req = Empty.Request()
        self.reader = rosbag2_py.SequentialReader()
        self.publisher = self.create_publisher(SceneUpdate,self.det_topic,10)
    
        # Create member variables
        self.results_dict = dict()

        # If results directory doesn't exist, create it
        if not os.path.exists(self.results_dir):
            os.mkdir(self.results_dir)

    def tracker_callback(self, msg):
        self.msg_count+=1
        self.get_logger().info("Tracker msg #%d received" % self.msg_count)
        self.get_logger().info("CALLBACK: has %i tracks\n" % (len(msg.tracks)))

        for kv in msg.metadata:
            if kv.key == "sample_token":
                if (kv.value) not in self.results_dict["results"].keys():
                    self.results_dict["results"][kv.value] = []

        # Populate dictionary entry
        for track in msg.tracks:

            if track.class_string in ['void_ignore']:
                continue

            track_dict = dict()
            track_dict["sample_token"] = msg.metadata[0].value
            track_dict["translation"] = [track.pose.pose.position.x, track.pose.pose.position.y, track.pose.pose.position.z]
            track_dict["size"] = [track.bbox.size.y, track.bbox.size.x, track.bbox.size.z] 
            track_dict["rotation"] = [track.pose.pose.orientation.w, track.pose.pose.orientation.x, track.pose.pose.orientation.y, track.pose.pose.orientation.z]
            track_dict["velocity"] = [0., 0.]
            track_dict["attribute_name"] = ''
            track_dict["tracking_score"] = track.track_confidence
            track_dict["tracking_name"] = track.class_string
            track_dict["tracking_id"] = int(track.track_id) 

            self.results_dict["results"][msg.metadata[0].value].append(track_dict)

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
            self.future = self.reconf_tracker_client.call_async(self.empty_req)
            rclpy.spin_until_future_complete(self, self.future)

            # Initialize results
            self.results_dict = dict()
            self.add_result_metadata()

            # Iterate through scene, messages

            for scene in self.split:
                self.get_logger().info("Computing tracking results for scene %s" % (scene))

                # Load .mcap file for this scene
                self.get_logger().info("Loading %s/%s%s/%s%s_0.mcap" % (self.mcap_dir, scene, self.lidar_det_string, scene, self.lidar_det_string))
                storage_options = rosbag2_py.StorageOptions(
                    uri="%s/%s%s/%s%s_0.mcap" % (self.mcap_dir, scene, self.lidar_det_string, scene, self.lidar_det_string),
                    storage_id='mcap')
                converter_options = rosbag2_py.ConverterOptions('', '')
                self.reader.open(storage_options, converter_options)

                # Deal with type names
                topic_types = self.reader.get_all_topics_and_types()

                def typename(topic_name):
                    for topic_type in topic_types:
                        if topic_type.name == topic_name:
                            return topic_type.type
                    raise ValueError(f"topic {topic_name} not in bag")

                # Reset tracker
                self.future = self.reset_tracker_client.call_async(self.empty_req)
                rclpy.spin_until_future_complete(self, self.future)



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