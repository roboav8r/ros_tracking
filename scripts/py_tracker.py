#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from tracking_msgs.msg import Tracks3D, Track3D, Detections3D, Detection3D

from ros_tracking.track_management import CreateTrack
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
objects = dict()
objects['trk_delete_conf'] = 0.25
objects['trk_delete_timeout'] = 3.0
objects['trk_delete_missed_det'] = 4

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

        # Create sensor subscriber objects
        self.subscription = self.create_subscription(
            eval(sensor_params['nuscenes-megvii']['msg_type']),
            sensor_params['nuscenes-megvii']['topic'],
            self.det_callback, 10
        )

    def det_callback(self, det_array_msg):
        self.dets = []
       
        # POPULATE detections list from detections message
        self.get_logger().info("DETECT: received %i detections" % (len(det_array_msg.detections)))
        for det in det_array_msg.detections:
            self.dets.append(GraphDet(det_array_msg,det))
        self.get_logger().info("DETECT: formatted %i detections \n" % (len(self.dets)))

        # PROPAGATE existing tracks

        # ASSIGN detections to tracks

        # UPDATE tracks with assigned detections

        # DELETE unmatched tracks, as appropriate

        # CREATE tracks from unmatched detections, as appropriate
        while self.dets:
            if (len(self.dets)-1) in self.det_asgn_idx: # If detection at end of list is matched, remove it
                self.dets.pop()

            elif CreateTrack(self.dets[-1]): # Check to see if track should be created from detection
                self.trks.append(GraphTrack(self.trk_id_count,self.dets.pop()))
                self.trk_id_count += 1

            else: # Otherwise, remove the detection
                self.dets.pop()
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