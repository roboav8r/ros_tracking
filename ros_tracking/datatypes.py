import gtsam
import numpy as np

from rclpy.time import Time

from tracking_msgs.msg import Tracks3D, Track3D, Detections3D, Detection3D

# Special data types to take ROS msg information and format for graph usage
class GraphDet():
    def __init__(self, dets_msg, det_msg):
        # Admin
        self.timestamp = Time.from_msg(dets_msg.header.stamp)
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
        self.time_updated = graph_det.timestamp
        self.missed_det = 0
        self.dt = 0.

        # Get semantic parameters
        # TODO - compute this based on detection probability
        self.prob_existence = graph_det.class_conf # probability of existence 

        # Initialize spatial state
        self.pos = graph_det.pos
        self.vel = np.array([[0.],[0.],[0.]])
        self.cov = np.diag([.05,.05,.1, .1, .1, .1]) # TODO - compute cov from sensor position covariance, object velocity covariance

        # Kalman filter for this object
        self.kf = gtsam.KalmanFilter(6)
        self.spatial_state = self.kf.init(np.vstack((self.pos, self.vel)), self.cov)

        # Process model parameters
        self.vel_variance = np.array([[0.25],[0.25],[0.1]]) # In tracker frame TODO - get this from object, attribute
        self.proc_model = np.diag(np.ones(6))
        self.proc_noise = gtsam.noiseModel.Diagonal.Sigmas([1,1,1,1,1,1])

    def compute_proc_model(self,dt):
        self.proc_model[0,3], self.proc_model[1,4], self.proc_model[2,5]  = dt, dt, dt

    def compute_proc_noise(self,dt):
        self.proc_noise = gtsam.noiseModel.Diagonal.Sigmas([0.25*self.vel_variance[0]**2*dt**4,
                                                            0.25*self.vel_variance[1]**2*dt**4,
                                                            0.25*self.vel_variance[2]**2*dt**4,
                                                            0.5*self.vel_variance[0]**2*dt**2,
                                                            0.5*self.vel_variance[1]**2*dt**2,
                                                            0.5*self.vel_variance[2]**2*dt**2])
    def propagate(self, t):
        self.dt = (Time.from_msg(t) - self.timestamp).nanoseconds/10**9
        self.timestamp = Time.from_msg(t)
        self.compute_proc_model(self.dt)
        self.compute_proc_noise(self.dt)
        self.spatial_state = self.kf.predict(self.spatial_state,self.proc_model,np.zeros((6,6)),np.zeros((6,1)),self.proc_noise)
        self.matched = False

    def update(self, det, obs_model, obs_noise):
        self.spatial_state = self.kf.update(self.spatial_state, obs_model, det.pos, obs_noise)
        self.timestamp = det.timestamp
        self.time_updated = det.timestamp
        self.missed_det = 0
        # if det.size is not [0,0,0]: # Only update box size if it's available
        #     self.obj_depth, self.obj_width, self.obj_height = det.obj_depth, det.obj_width, det.obj_height
        # self.transform = det.trk_transform