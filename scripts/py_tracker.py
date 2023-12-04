#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from tracking_msgs.msg import Tracks3D, Track3D, Detections3D, Detection3D

from ros_tracking.assignment import ComputeAssignment
from ros_tracking.track_management import CreateTracks, DeleteTracks
from ros_tracking.datatypes import GraphDet, GraphTrack

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
# tracker_params['pub_topic'] = "tracks"
# tracker_params['msg_type'] = 'Tracks3D'

# Object parameters
# TODO - add to yaml file

class Tracker(Node):
    def __init__(self, sensor_params):
        super().__init__('tracker')

        # Tracks and detections
        self.trk_id_count = 0
        self.dets = []
        self.trks = []

        # Assignment
        self.cost_matrix = np.empty(0)
        self.det_asgn_idx = [] 
        self.trk_asgn_idx = []
        self.asgn_thresh = tracker_params['asgn_thresh']

        # Track management
        # TODO - read this in from a file, OR assign to object parameters
        self.trk_delete_prob = 0.35
        self.trk_delete_timeout = 0.75
        self.trk_delete_missed_det = 2

        # Create sensor subscriber objects
        self.subscription = self.create_subscription(
            eval(sensor_params['nuscenes-megvii']['msg_type']),
            sensor_params['nuscenes-megvii']['topic'],
            self.det_callback, 10
        )

    def propagate_tracks(self, det_array_msg):
        for trk in self.trks:
            trk.propagate(det_array_msg.header.stamp)

    def update_tracks(self):
        for det_idx, trk_idx in zip(self.det_asgn_idx, self.trk_asgn_idx):
            self.trks[trk_idx].update(self.dets[det_idx])

    def det_callback(self, det_array_msg):
        self.dets = []
       
        # POPULATE detections list from detections message
        self.get_logger().info("DETECT: received %i detections" % (len(det_array_msg.detections)))
        for det in det_array_msg.detections:
            self.dets.append(GraphDet(det_array_msg,det))
        self.get_logger().info("DETECT: formatted %i detections \n" % (len(self.dets)))

        # PROPAGATE existing tracks
        self.propagate_tracks(det_array_msg)

        # ASSIGN detections to tracks
        ComputeAssignment(self)
        self.get_logger().info("ASSIGN: cost matrix has shape %lix%li \n" % (self.cost_matrix.shape[0],self.cost_matrix.shape[1]))
        self.get_logger().info("ASSIGN: det assignment vector has length %li \n" % (len(self.det_asgn_idx)))
        self.get_logger().info("ASSIGN: trk assignment vector has length %li \n" % (len(self.trk_asgn_idx)))

        # UPDATE tracks with assigned detections
        self.update_tracks()

        # DELETE unmatched tracks, as appropriate
        for i, trk in enumerate(self.trks):
            if i not in self.trk_asgn_idx: # If track is unmatched
                trk.missed_det +=1 # Increment missed detection counter # TODO - OR update existence probability
        DeleteTracks(self)
        self.get_logger().info("DELETE: have %i tracks, %i detections \n" % (len(self.trks), len(self.dets)))

        # CREATE tracks from unmatched detections, as appropriate
        CreateTracks(self)
        self.get_logger().info("CREATE: have %i tracks, %i detections \n" % (len(self.trks), len(self.dets)))
            
        # OUTPUT tracker results


def main(args=None):
    rclpy.init(args=args)

    # Create tracker object
    tracker = Tracker(sensors)

    rclpy.spin(tracker)

    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()