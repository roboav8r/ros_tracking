import gtsam
import numpy as np
from tracking_msgs.msg import Tracks3D, Track3D, Detections3D, Detection3D

# Special data types to take ROS msg information and format for graph usage
class GraphDet():
    def __init__(self, dets_msg, det_msg):
        # Admin
        self.timestamp = dets_msg.header
        # TODO - id? sym?
        
        # Spatial properties
        # TODO - gtsam pose, vector types
        self.pose = det_msg.pose
        self.pos = np.array([[det_msg.pose.position.x], [det_msg.pose.position.y], [det_msg.pose.position.z]])
        self.size = det_msg.bbox.size

        # Semantic Properties
        # TODO - GTSAM distribution
        # self.class_id 
        self.class_string = det_msg.class_string
        self.class_conf = det_msg.class_confidence
        self.attribute = det_msg.attribute


class GraphTrack():
    def __init__(self, id, graph_det):
        # Admin
        self.trk_id = id
        self.timestamp = graph_det.timestamp
        self.time_created = graph_det.timestamp
        self.last_updated = graph_det.timestamp
        self.missed_det = 0

        # Initialize spatial state
        self.pos = graph_det.pos
        self.vel = np.array([[0.],[0.],[0.]])
        self.cov = np.diag([.05,.05,.1, .1, .1, .1]) # TODO - compute cov from sensor position covariance, object velocity covariance

        # Kalman filter for this object
        self.kf = gtsam.KalmanFilter(6)
        self.state = self.kf.init(np.vstack((self.pos, self.vel)), self.cov)

        # Process model matrices
        # self.proc_model = np.diag(np.ones(6))
        # self.proc_noise = gtsam.noiseModel.Diagonal.Sigmas([1,1,1,1,1,1])

        # TODO - covariance/uncertainty for pose