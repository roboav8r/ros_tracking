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
        self.size = np.array([[det_msg.bbox.size.x], [det_msg.bbox.size.y], [det_msg.bbox.size.z]])

        # Semantic Properties
        # TODO - GTSAM distribution
        # self.class_id 
        self.class_string = det_msg.class_string
        # self.class_conf = det_msg.class_confidence
        # self.attribute = det_msg.attribute


class GraphTrack():
    def __init__(self, id, graph_det, prob_class_det, det_idx_map):
        # Admin
        self.trk_id = id
        self.timestamp = graph_det.timestamp
        self.time_created = graph_det.timestamp
        self.time_updated = graph_det.timestamp
        self.dt = 0.

        # Get semantic parameters
        self.class_dist = gtsam.DiscreteDistribution(prob_class_det.likelihood(det_idx_map[graph_det.class_string]))

        # Initialize spatial state
        # Linear
        self.pos = graph_det.pos
        self.vel = np.array([[0.],[0.],[0.]])
        # Angular
        self.orientation = gtsam.Rot3(graph_det.pose.orientation.w,
                                      graph_det.pose.orientation.x,
                                      graph_det.pose.orientation.y,
                                      graph_det.pose.orientation.z)
        self.rot = np.array([[self.orientation.rpy()[0]],[self.orientation.rpy()[1]],[self.orientation.rpy()[2]]])
        self.rot_rate = np.array([[0.],[0.],[0.]])
        # Size
        self.size = graph_det.size # TODO - check if there is a bounding box or a point detection
        self.cov = np.diag([.05,.05,.1, .1, .1, .1, .01, .01, .05, .1, .1, .1, .001, .001, .001]) # TODO - compute cov from sensor position covariance, object velocity covariance

        # Kalman filter for this object
        self.kf = gtsam.KalmanFilter(15)
        self.spatial_state = self.kf.init(np.vstack((self.pos, self.vel, self.rot, self.rot_rate, self.size)), self.cov)

        # Process model parameters
        self.vel_variance = np.array([[0.25],[0.25],[0.1]]) # In tracker frame TODO - get this from object, attribute
        self.rot_rate_variance = np.array([[0.25],[0.25],[0.1]]) # In tracker frame TODO - get this from object, attribute
        self.size_variance = np.array([[0.005],[0.005],[0.005]])
        self.proc_model = np.diag(np.ones(15))
        self.proc_noise = gtsam.noiseModel.Diagonal.Sigmas([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1])

    def compute_proc_model(self,dt):
        #TODO - sine and cosine terms for planar velocity
        self.proc_model[0,3], self.proc_model[1,4], self.proc_model[2,5]  = dt, dt, dt
        self.proc_model[6,9], self.proc_model[7,10], self.proc_model[8,11]  = dt, dt, dt

    def compute_proc_noise(self,dt):
        self.proc_noise = gtsam.noiseModel.Diagonal.Sigmas([0.25*self.vel_variance[0]**2*dt**4,
                                                            0.25*self.vel_variance[1]**2*dt**4,
                                                            0.25*self.vel_variance[2]**2*dt**4,
                                                            0.5*self.vel_variance[0]**2*dt**2,
                                                            0.5*self.vel_variance[1]**2*dt**2,
                                                            0.5*self.vel_variance[2]**2*dt**2,
                                                            0.25*self.rot_rate_variance[0]**2*dt**4,
                                                            0.25*self.rot_rate_variance[1]**2*dt**4,
                                                            0.25*self.rot_rate_variance[2]**2*dt**4,
                                                            0.5*self.rot_rate_variance[0]**2*dt**2,
                                                            0.5*self.rot_rate_variance[1]**2*dt**2,
                                                            0.5*self.rot_rate_variance[2]**2*dt**2,
                                                            self.size_variance[0],
                                                            self.size_variance[1],
                                                            self.size_variance[2]])
    def propagate(self, t):
        self.dt = (Time.from_msg(t) - self.timestamp).nanoseconds/10**9
        self.timestamp = Time.from_msg(t)
        self.compute_proc_model(self.dt)
        self.compute_proc_noise(self.dt)
        self.spatial_state = self.kf.predict(self.spatial_state,self.proc_model,np.zeros((15,15)),np.zeros((15,1)),self.proc_noise)

    def update(self, det, obs_model, obs_noise, prob_class_det, det_idx_map):
        self.spatial_state = self.kf.update(self.spatial_state, obs_model, np.vstack((det.pos, det.size)), obs_noise)
        
        # TODO make helper function for rpy->quat
        cr = np.cos(0.5*self.spatial_state.mean()[6])
        sr = np.sin(0.5*self.spatial_state.mean()[6])
        cp = np.cos(0.5*self.spatial_state.mean()[7])
        sp = np.sin(0.5*self.spatial_state.mean()[7])
        cy = np.cos(0.5*self.spatial_state.mean()[8])
        sy = np.sin(0.5*self.spatial_state.mean()[8])

        self.orientation = gtsam.Rot3(cr*cp*cy + sr*sp*sy,
                                      sr*cp*cy - cr*sp*sy,
                                      cr*sp*cy + sr*cp*sy,
                                      cr*cp*sy - sr*sp*cy)
        self.timestamp = det.timestamp
        self.time_updated = det.timestamp
        self.missed_det = 0
        self.class_dist = gtsam.DiscreteDistribution(prob_class_det.likelihood(det_idx_map[det.class_string])*self.class_dist)
        # if det.size is not [0,0,0]: # Only update box size if it's available
        #     self.obj_depth, self.obj_width, self.obj_height = det.obj_depth, det.obj_width, det.obj_height
        # self.transform = det.trk_transform