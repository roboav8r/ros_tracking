#!/usr/bin/env python3

import gtsam
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from foxglove_msgs.msg import SceneUpdate, SceneEntity
from tracking_msgs.msg import Tracks3D, Track3D, Detections3D, Detection3D

from ros_tracking.datatypes import GraphDet, GraphTrack
from ros_tracking.assignment import ComputeAssignment
from ros_tracking.track_management import CreateTracks, DeleteTracks
from ros_tracking.output import PublishTracks, PublishScene
from ros_tracking.sensors import CreateSensorModels

# Sensor parameters
# TODO - add to yaml file
sensors = dict()
sensors['nuscenes-megvii'] = dict()
sensors['nuscenes-megvii']['topic'] = "detections"
sensors['nuscenes-megvii']['msg_type'] = "Detections3D"
sensors['nuscenes-megvii']['pos_noise'] = [.2, .2, .2]

# Tracker parameters
# TODO - add to yaml file
tracker_params = dict()
tracker_params['frame_id'] = 'map'
tracker_params['asgn_thresh'] = 4.0
tracker_params['trk_pub_topic'] = "tracks"
tracker_params['trk_msg_type'] = 'Tracks3D'
tracker_params['scene_pub_topic'] = "tracks_scene"
tracker_params['scene_msg_type'] = 'SceneUpdate'

# Object parameters
# TODO - add to yaml file

class Tracker(Node):
    def __init__(self, sensor_params):
        super().__init__('tracker')

        # Tracker properties
        self.frame_id = tracker_params['frame_id']

        # Generate sensor models from .yaml, initialize empty callback messages
        self.dets_msg = Detections3D()
        CreateSensorModels(self)          

        # TODO - Get tracker parameters from config file
        self.class_idx_map = dict()
        self.object_classes =  ['false_detection', 'void_ignore', 'bicycle', 'bus', 'car', 'motorcycle', 'pedestrian', 'trailer', 'truck']
        for idx, class_name in enumerate(self.object_classes):
            self.class_idx_map[class_name] = idx

        # Track management
        # TODO - read this in from a file, OR assign to object parameters
        self.trk_delete_prob = 0.35
        self.trk_delete_timeout = 0.75
        self.trk_delete_missed_det = 2

        # Track and detection objects
        self.trk_id_count = 0
        self.dets = []
        self.trks = []

        # Assignment
        self.cost_matrix = np.empty(0)
        self.det_asgn_idx = [] 
        self.trk_asgn_idx = []
        self.asgn_thresh = tracker_params['asgn_thresh']

        # Create publisher objects and empty messages
        self.trks_msg = Tracks3D()
        self.scene_msg = SceneUpdate()
        self.track_pub = self.create_publisher(
            eval(tracker_params['trk_msg_type']),
            tracker_params['trk_pub_topic'],
            10
        )
        self.scene_pub = self.create_publisher(
            eval(tracker_params['scene_msg_type']),
            tracker_params['scene_pub_topic'],
            10
        )

    def propagate_tracks(self):
        for trk in self.trks:
            trk.propagate(self.dets_msg.header.stamp)

    def update_tracks(self, obs_mdl, obs_var, prob_class_det, det_idx_map):
        for det_idx, trk_idx in zip(self.det_asgn_idx, self.trk_asgn_idx):
            self.trks[trk_idx].update(self.dets[det_idx], obs_mdl, obs_var, prob_class_det, det_idx_map)

    def det_callback(self, det_array_msg, obs_model, obs_variance, prob_class_det, det_idx_map):
        self.dets_msg = det_array_msg
        self.dets = []
       
        # POPULATE detections list from detections message
        self.get_logger().info("DETECT: received %i detections" % (len(self.dets_msg.detections)))
        for det in self.dets_msg.detections:
            self.dets.append(GraphDet(self.dets_msg,det))
        self.get_logger().info("DETECT: formatted %i detections \n" % (len(self.dets)))

        # PROPAGATE existing tracks
        self.propagate_tracks()

        # ASSIGN detections to tracks
        ComputeAssignment(self)
        self.get_logger().info("ASSIGN: cost matrix has shape %lix%li \n" % (self.cost_matrix.shape[0],self.cost_matrix.shape[1]))
        self.get_logger().info("ASSIGN: det assignment vector has length %li \n" % (len(self.det_asgn_idx)))
        self.get_logger().info("ASSIGN: trk assignment vector has length %li \n" % (len(self.trk_asgn_idx)))

        # UPDATE tracks with assigned detections
        self.update_tracks(obs_model, obs_variance, prob_class_det, det_idx_map)

        # DELETE unmatched tracks, as appropriate
        for i, trk in enumerate(self.trks):
            if i not in self.trk_asgn_idx: # If track is unmatched
                trk.class_dist = gtsam.DiscreteDistribution(prob_class_det.likelihood(det_idx_map['missed_detection']))
                trk.missed_det +=1 # Increment missed detection counter 
        DeleteTracks(self)
        self.get_logger().info("DELETE: have %i tracks, %i detections \n" % (len(self.trks), len(self.dets)))

        # CREATE tracks from unmatched detections, as appropriate
        CreateTracks(self, prob_class_det, det_idx_map)
        self.get_logger().info("CREATE: have %i tracks, %i detections \n" % (len(self.trks), len(self.dets)))
            
        # OUTPUT tracker results
        PublishTracks(self)
        PublishScene(self)


def main(args=None):
    rclpy.init(args=args)

    # Create tracker object
    tracker = Tracker(sensors)

    rclpy.spin(tracker)

    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()