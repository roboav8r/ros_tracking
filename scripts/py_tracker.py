#!/usr/bin/env python3

import gtsam
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Empty
from foxglove_msgs.msg import SceneUpdate, SceneEntity
from tracking_msgs.msg import Tracks3D, Track3D, Detections3D, Detection3D

from ros_tracking.datatypes import GraphDet, GraphTrack
from ros_tracking.assignment import ComputeAssignment
from ros_tracking.track_management import CreateTracks, DeleteTracks
from ros_tracking.output import PublishTracks, PublishScene
from ros_tracking.sensors import CreateSensorModels
from ros_tracking.tracker import ConfigureTracker

# Object parameters
# TODO - add to yaml file

class Tracker(Node):
    def __init__(self):
        super().__init__('tracker')

        # Configure tracker from .yaml
        ConfigureTracker(self)

        # Generate sensor models from .yaml, initialize empty callback messages
        self.dets_msg = Detections3D()
        CreateSensorModels(self)          

        # Track and detection variables
        self.trk_id_count = 0
        self.dets = []
        self.trks = []

        # Assignment variables
        self.cost_matrix = np.empty(0)
        self.det_asgn_idx = [] 
        self.trk_asgn_idx = []

        # Create publisher objects and empty messages
        self.trks_msg = Tracks3D()
        self.scene_msg = SceneUpdate()

        # Declare services
        self.reset_srv = self.create_service(Empty, 'reset_tracker', self.reset_tracker)

    def reset_tracker(self,req, resp):
        # self.get_logger().info("Resetting tracker")

        # Clear track, detection, and assignment variables
        self.dets = []
        self.trks = []

        self.cost_matrix = np.empty(0)
        self.det_asgn_idx = [] 
        self.trk_asgn_idx = []       

        return resp

    def propagate_tracks(self):
        for trk in self.trks:
            trk.propagate(self.dets_msg.header.stamp)

    def update_tracks(self, obs_mdl, obs_var, prob_class_label, det_params):
        for det_idx, trk_idx in zip(self.det_asgn_idx, self.trk_asgn_idx):
            self.trks[trk_idx].update(self.dets[det_idx], obs_mdl, obs_var, prob_class_label, det_params)

    def det_callback(self, det_array_msg, obs_model, obs_variance, prob_class_label, det_params):
        self.dets_msg = det_array_msg
        metadata = self.dets_msg.detections[0].metadata
        self.dets = []
       
        # POPULATE detections list from detections message
        self.get_logger().info("DETECT: received %i detections" % (len(self.dets_msg.detections)))
        for det in self.dets_msg.detections:
            self.dets.append(GraphDet(self.dets_msg,det))
        # self.get_logger().info("DETECT: formatted %i detections \n" % (len(self.dets)))

        # PROPAGATE existing tracks
        self.propagate_tracks()

        # ASSIGN detections to tracks
        ComputeAssignment(self)
        self.get_logger().info("ASSIGN: cost matrix has shape %lix%li \n" % (self.cost_matrix.shape[0],self.cost_matrix.shape[1]))
        self.get_logger().info("ASSIGN: det assignment vector has length %li \n" % (len(self.det_asgn_idx)))
        self.get_logger().info("ASSIGN: trk assignment vector has length %li \n" % (len(self.trk_asgn_idx)))

        # UPDATE tracks with assigned detections
        self.update_tracks(obs_model, obs_variance, prob_class_label, det_params)

        # UPDATE unmatched tracks (missed detections)
        for i, trk in enumerate(self.trks):
            if i not in self.trk_asgn_idx: # If track is unmatched, handle it as a missed detection

                p_missed = det_params['p_missed_det'][trk.class_dist.argmax()]

                # print("MISSED DET UPDATE")
                # print("Track conf was %s, class has missed det prob %s" % (trk.track_conf, p_missed))
                # print("Update distribution is %s" % gtsam.DiscreteDistribution([det_params['det_sym'],2],[1-p_missed, p_missed]))
                
                # trk.class_dist = gtsam.DiscreteDistribution(prob_class_label.likelihood(det_params['missed_detection']))
                # trk.track_conf = gtsam.DiscreteDistribution(gtsam.DiscreteDistribution([det_params['det_sym'],2],[1-p_missed, p_missed])*trk.track_conf)
                trk.track_conf = gtsam.DiscreteDistribution([gtsam.symbol('e',trk.trk_id),2],[(1-p_missed)*trk.track_conf(0),p_missed*trk.track_conf(1)])
                # print("Now it's %s" % (trk.track_conf))
                trk.metadata = metadata
                trk.n_missed += 1
        DeleteTracks(self)
        # self.get_logger().info("DELETE: have %i tracks, %i detections \n" % (len(self.trks), len(self.dets)))

        # CREATE tracks from unmatched detections, as appropriate
        CreateTracks(self, prob_class_label, det_params)
        # self.get_logger().info("CREATE: have %i tracks, %i detections \n" % (len(self.trks), len(self.dets)))

        # OUTPUT tracker results
        self.get_logger().info("PUBLISH: have %i tracks, %i detections \n" % (len(self.trks), len(self.dets)))
        for pub_name in self.pubs:
            # print(pub_name)
            # print(self.pubs[pub_name])
            exec('%s(self,\'%s\')' % (self.pubs[pub_name]['routine'],pub_name))
        # PublishTracks(self)
        # PublishScene(self)


def main(args=None):
    rclpy.init(args=args)

    # Create tracker object
    tracker = Tracker()

    rclpy.spin(tracker)

    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()