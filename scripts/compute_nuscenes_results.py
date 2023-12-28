#!/usr/bin/env python3

import os
import time 
import json

from nuscenes.can_bus.can_bus_api import NuScenesCanBus
from nuscenes.eval.common.utils import quaternion_yaw
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.nuscenes import NuScenes
import nuscenes.utils.splits as nuscenes_splits

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.wait_for_message import wait_for_message

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from std_srvs.srv import Empty
from foxglove_msgs.msg import SceneUpdate, SceneEntity
from tracking_msgs.msg import Detections3D, Tracks3D


class ComputeNuscenesResults(Node):
    def __init__(self):
        super().__init__('compute_nuscenes_node')

        # Get parameters from yaml file
        self.declare_parameter('nuscenes_dir', rclpy.Parameter.Type.STRING )
        self.declare_parameter('results_dir', rclpy.Parameter.Type.STRING )
        self.declare_parameter('mcap_dir', rclpy.Parameter.Type.STRING )
        # self.declare_parameter('dataset_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('lidar_detector', rclpy.Parameter.Type.STRING)
        self.declare_parameter('split_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('track_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('det_topic', rclpy.Parameter.Type.STRING)
        self.nuscenes_dir = self.get_parameter('nuscenes_dir').get_parameter_value().string_value
        self.results_dir = self.get_parameter('results_dir').get_parameter_value().string_value
        self.mcap_dir = self.get_parameter('mcap_dir').get_parameter_value().string_value
        # self.dataset_name = self.get_parameter('dataset_name').get_parameter_value().string_value
        self.lidar_detector = self.get_parameter('lidar_detector').get_parameter_value().string_value
        self.split_name = self.get_parameter('split_name').get_parameter_value().string_value
        self.track_topic = self.get_parameter('track_topic').get_parameter_value().string_value
        self.det_topic = self.get_parameter('det_topic').get_parameter_value().string_value
        self.lidar_det_string = "-" + self.lidar_detector if self.lidar_detector else ""
        
        # Create ROS objects
        self.reset_tracker_client = self.create_client(Empty, "reset_tracker")
        self.reset_req = Empty.Request()
        self.publisher = self.create_publisher(SceneUpdate,self.det_topic,10)
        self.reader = rosbag2_py.SequentialReader()

        # Member variables
        times = []
        self.msg_count=0
        # self.debug_dict = dict()

        # Create nuscenes dataset object
        # nusc = NuScenes(version=self.dataset_name, dataroot=str(self.nuscenes_dir), verbose=True)

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
        split = eval('nuscenes_splits.' + self.split_name)

        for scene in split: # for scene in split
            self.get_logger().info("Computing tracking results for scene %s" % (scene))

            # Reset tracker
            self.future = self.reset_tracker_client.call_async(self.reset_req)
            rclpy.spin_until_future_complete(self, self.future)

            # Load bag
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


            while self.reader.has_next():
                topic, data, timestamp = self.reader.read_next()

                if topic==self.det_topic:
                    # self.get_logger().info("Topic: %s" % (topic))                  

                    msg_type = get_message(typename(topic))
                    msg = deserialize_message(data, msg_type)
                    # self.get_logger().info("Msg: %s\n" % (msg))

                    # Send the detection message
                    self.get_logger().info("Sending message")
                    self.publisher.publish(msg)
                    tic = time.process_time()

                    # wait for the track response from the tracker
                    ret, trk_msg = wait_for_message(Tracks3D, self, self.track_topic)
                    toc = time.process_time()
                    times.append(toc-tic)
                    self.tracker_callback(trk_msg)

        # Save results
        with open(self.results_dir + "/results.json", "w") as outfile:
            json.dump(self.results_dict, outfile, indent=2)

        # with open(self.results_dir + "/debug.json", "w") as outfile:
        #     json.dump(self.debug_dict, outfile, indent=2)

        

        self.get_logger().info("Times: %s" % times)

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

            track_dict = dict()
            track_dict["sample_token"] = msg.metadata[0].value
            track_dict["translation"] = [track.pose.pose.position.x, track.pose.pose.position.y, track.pose.pose.position.z]
            track_dict["size"] = [track.bbox.size.y, track.bbox.size.x, track.bbox.size.z] 
            track_dict["rotation"] = [track.pose.pose.orientation.w, track.pose.pose.orientation.x, track.pose.pose.orientation.y, track.pose.pose.orientation.z]
            track_dict["velocity"] = [0., 0.]
            track_dict["attribute_name"] = '' # TODO - add attributes
            # track_dict["tracking_score"] = float(track.class_confidence)
            track_dict["tracking_score"] = track.track_confidence
            track_dict["tracking_name"] = track.class_string
            track_dict["tracking_id"] = int(track.track_id) #track.track_id

            # self.debug_dict[self.msg_count]['trk_ids'].append(int(entity.id))
            self.results_dict["results"][msg.metadata[0].value].append(track_dict)

def main(args=None):
    rclpy.init(args=args)

    # Create tracker object
    compute_nusc_res = ComputeNuscenesResults()

    rclpy.spin(compute_nusc_res)

    compute_nusc_res.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()