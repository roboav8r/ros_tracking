#!/usr/bin/env python3

import os
import time 
import json

from nuscenes.can_bus.can_bus_api import NuScenesCanBus
from nuscenes.eval.common.utils import quaternion_yaw
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.splits import create_splits_scenes

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message

from std_srvs.srv import Empty
from foxglove_msgs.msg import SceneUpdate, SceneEntity
# from tracking_msgs.msg import Detections3D, Tracks3D


class ComputeNuscenesResults(Node):
    def __init__(self):
        super().__init__('compute_nuscenes_node')

        # Get parameters from yaml file
        self.declare_parameter('nuscenes_dir', rclpy.Parameter.Type.STRING )
        self.declare_parameter('results_dir', rclpy.Parameter.Type.STRING )
        self.declare_parameter('mcap_dir', rclpy.Parameter.Type.STRING )
        self.declare_parameter('dataset_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('lidar_detector', rclpy.Parameter.Type.STRING)
        self.declare_parameter('split_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('track_topic', rclpy.Parameter.Type.STRING)
        self.nuscenes_dir = self.get_parameter('nuscenes_dir').get_parameter_value().string_value
        self.results_dir = self.get_parameter('results_dir').get_parameter_value().string_value
        self.mcap_dir = self.get_parameter('mcap_dir').get_parameter_value().string_value
        self.dataset_name = self.get_parameter('dataset_name').get_parameter_value().string_value
        self.lidar_detector = self.get_parameter('lidar_detector').get_parameter_value().string_value
        self.split_name = self.get_parameter('split_name').get_parameter_value().string_value
        self.track_topic = self.get_parameter('track_topic').get_parameter_value().string_value
        self.lidar_det_string = "-" + self.lidar_detector if self.lidar_detector else ""
        
        # Create ROS objects
        self.reset_tracker_client = self.create_client(Empty, "reset_tracker")
        self.reset_req = Empty.Request()
        # self.writer = rosbag2_py.SequentialWriter()
        self.subscription = self.create_subscription(SceneUpdate,self.track_topic,self.tracker_callback,10)
        self.subscription

        # Create nuscenes dataset object
        nusc = NuScenes(version=self.dataset_name, dataroot=str(self.nuscenes_dir), verbose=True)

        # Initialize member variables
        self.results_dict = dict()
        self.results_dict["meta"] = dict()
        self.results_dict["meta"]["use_camera"] = False
        self.results_dict["meta"]["use_lidar"] = True
        self.results_dict["meta"]["use_radar"] = False
        self.results_dict["meta"]["use_map"] = False
        self.results_dict["meta"]["use_external"] = False
        self.results_dict["results"] = dict()

        # for scene in dataset
        print('TODO get split')

        for scene in nusc.scene: # for scene in split
            self.get_logger().info("Computing tracking results for scene %s" % (scene['name']))

            # Reset tracker
            self.future = self.reset_tracker_client.call_async(self.reset_req)
            rclpy.spin_until_future_complete(self, self.future)

            # Start mcap playback of detection data
            bag_play_str = "ros2 bag play %s/%s%s/%s%s_0.mcap" % (self.mcap_dir, scene['name'], self.lidar_det_string, scene['name'], self.lidar_det_string)
            returned_val = os.system(bag_play_str)

            # Wait for last message to be completed, then move to next scene
            # TODO
            self.get_logger().info("Bag play command returned %s" % (returned_val))
            # if returned_val:
            # os.system(b'\x03') # ctrl-c to stop recording

        # Save results
        with open(self.results_dir + "/results.json", "w") as outfile:
            json.dump(self.results_dict, outfile, indent=2)

    def tracker_callback(self, msg):

        # Populate dictionary entry
        for entity in msg.entities:

        # sample_result {
        #     "sample_token":   <str>         -- Foreign key. Identifies the sample/keyframe for which objects are detected.
        #     "translation":    <float> [3]   -- Estimated bounding box location in meters in the global frame: center_x, center_y, center_z.
        #     "size":           <float> [3]   -- Estimated bounding box size in meters: width, length, height.
        #     "rotation":       <float> [4]   -- Estimated bounding box orientation as quaternion in the global frame: w, x, y, z.
        #     "velocity":       <float> [2]   -- Estimated bounding box velocity in m/s in the global frame: vx, vy.
        #     "tracking_id":    <str>         -- Unique object id that is used to identify an object track across samples.
        #     "tracking_name":  <str>         -- The predicted class for this sample_result, e.g. car, pedestrian.
        #                                     Note that the tracking_name cannot change throughout a track.
        #     "tracking_score": <float>       -- Object prediction score between 0 and 1 for the class identified by tracking_name.
        #                                     We average over frame level scores to compute the track level score.
        #                                     The score is used to determine positive and negative tracks via thresholding.
        # }

            if (entity.metadata[3].value) not in self.results_dict["results"].keys():
                self.results_dict["results"][entity.metadata[3].value] = []

            track_dict = dict()
            track_dict["sample_token"] = entity.metadata[3].value
            track_dict["translation"] = [entity.cubes[0].pose.position.x, entity.cubes[0].pose.position.y, entity.cubes[0].pose.position.z]
            track_dict["size"] = [entity.cubes[0].size.x, entity.cubes[0].size.y, entity.cubes[0].size.z] 
            track_dict["rotation"] = [entity.cubes[0].pose.orientation.w, entity.cubes[0].pose.orientation.x, entity.cubes[0].pose.orientation.y, entity.cubes[0].pose.orientation.z]
            track_dict["velocity"] = [0., 0.]
            track_dict["attribute_name"] = entity.metadata[2].value
            track_dict["tracking_score"] = float(entity.metadata[1].value) #track.class_confidence
            track_dict["tracking_name"] = entity.metadata[0].value #track.class_string
            track_dict["tracking_id"] = entity.id #track.track_id
            self.results_dict["results"][entity.metadata[3].value].append(track_dict)



def main(args=None):
    rclpy.init(args=args)

    # Create tracker object
    compute_nusc_res = ComputeNuscenesResults()

    rclpy.spin(compute_nusc_res)

    compute_nusc_res.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()