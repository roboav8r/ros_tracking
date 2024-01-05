import gtsam
import numpy as np

from rclpy.time import Time

from tracking_msgs.msg import Tracks3D, Track3D, Detections3D, Detection3D
from diagnostic_msgs.msg import KeyValue

from ros_tracking.sensors import ComputeMatrixSpec

# Special data types to take ROS msg information and format for graph usage
class GraphDet():
    def __init__(self, dets_msg, det_msg):
        # Admin
        self.timestamp = Time.from_msg(dets_msg.header.stamp)
       
        # Spatial properties
        self.pose = det_msg.pose
        self.pos = np.array([[det_msg.pose.position.x], [det_msg.pose.position.y], [det_msg.pose.position.z]])
        self.size = np.array([[det_msg.bbox.size.x], [det_msg.bbox.size.y], [det_msg.bbox.size.z]])

        # Semantic Properties
        self.metadata = det_msg.metadata
        self.class_string = det_msg.class_string
        self.class_conf = det_msg.class_confidence


class GraphTrack():
    def __init__(self, tracker, graph_det, det_params):
        # Admin
        self.trk_id = tracker.trk_id_count
        self.timestamp = graph_det.timestamp
        self.time_created = graph_det.timestamp
        self.time_updated = graph_det.timestamp
        self.dt = 0.
        self.metadata = graph_det.metadata
        self.n_missed = 0
        self.n_matched = 1

        # Get semantic parameters
        det_idx = det_params[graph_det.class_string]['idx']
        self.class_dist = gtsam.DiscreteDistribution(det_params['p_class_label'].likelihood(det_idx))
        class_idx = self.class_dist.argmax()
        self.class_conf = graph_det.class_conf

        # Compute existence probability
        hist_ind = np.searchsorted( det_params['hist_bins'], float(graph_det.class_conf), side='right') -1
        self.track_conf = gtsam.DiscreteDistribution([gtsam.symbol('e',self.trk_id),2],[det_params['false_pos_hist'][class_idx][hist_ind],det_params['true_pos_hist'][class_idx][hist_ind]])

        # Initialize object type parameters and model
        self.obj_type = tracker.object_classes[class_idx]
        self.obj_params = tracker.object_properties[self.obj_type]

        # Initialize spatial state
        # Linear
        self.pos = graph_det.pos
        self.vel = np.array([[0.],[0.],[0.]])
        # Angular
        self.orientation = gtsam.Rot3(graph_det.pose.orientation.w,
                                      graph_det.pose.orientation.x,
                                      graph_det.pose.orientation.y,
                                      graph_det.pose.orientation.z)
        self.rpy = np.array([[self.orientation.rpy()[0]],[self.orientation.rpy()[1]],[self.orientation.rpy()[2]]])
        self.rpy_rate = np.array([[0.],[0.],[0.]])
        # Size
        self.size = graph_det.size # TODO - check if there is a bounding box or a point detection

        if self.obj_params['model_type'] == 'ab3dmot':
            self.n_states = 11
            self.cov = np.diag(np.concatenate((det_params['pos_var'][det_idx,:], det_params['yaw_var'][det_idx,:], det_params['size_var'][det_idx,:], det_params['conf_var'][det_idx,:], [10000],[10000],[10000]))) 

            # Kalman filter for this object
            self.kf = gtsam.KalmanFilter(11)
            self.spatial_state = self.kf.init(np.vstack((self.pos, self.orientation.rpy()[2], self.size, graph_det.class_conf, self.vel)), self.cov)

            # Build initial process model and noise
            self.proc_model = np.diag(np.ones(11))
            self.proc_noise = gtsam.noiseModel.Diagonal.Sigmas(self.obj_params['proc_var'])

        elif self.obj_params['model_type'] == 'ackermann':
            self.n_states = 11
            self.cov = np.diag(np.concatenate((det_params['pos_var'][det_idx,:], det_params['yaw_var'][det_idx,:], det_params['size_var'][det_idx,:], det_params['conf_var'][det_idx,:], [10000],[10000],[10000])))

            # Kalman filter for this object
            self.kf = gtsam.KalmanFilter(11)
            self.spatial_state = self.kf.init(np.vstack((self.pos, self.orientation.rpy()[2], self.size, graph_det.class_conf, self.vel)), self.cov)

            # Build initial process model and noise
            self.proc_model = np.diag(np.ones(11))
            self.proc_noise = gtsam.noiseModel.Diagonal.Sigmas(self.obj_params['proc_var'])

        else:
            self.n_states = 15
            self.cov = np.diag([.05,.05,.1, .1, .1, .1, .01, .01, .05, .1, .1, .1, .001, .001, .001]) # TODO - compute cov from sensor position covariance, object velocity covariance

            # Kalman filter for this object
            self.kf = gtsam.KalmanFilter(15)
            self.spatial_state = self.kf.init(np.vstack((self.pos, self.vel, self.rpy, self.rpy_rate, self.size)), self.cov)

            # Process model parameters
            self.vel_variance = np.array([[0.5],[0.1],[0.1]]) # In tracker frame TODO - get this from object, attribute
            self.rot_rate_variance = np.array([[0.05],[0.05],[0.25]]) # In tracker frame TODO - get this from object, attribute
            self.size_variance = np.array([[0.005],[0.005],[0.005]])
            self.proc_model = np.diag(np.ones(15))
            self.proc_noise = gtsam.noiseModel.Diagonal.Sigmas([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1])

    def compute_proc_model(self):
        if self.obj_params['model_type'] == 'ab3dmot':
            self.proc_model[0,8], self.proc_model[1,9], self.proc_model[2,10]  = self.dt, self.dt, self.dt

        elif self.obj_params['model_type'] == 'ackermann':
            self.proc_model[0,8], self.proc_model[1,9], self.proc_model[2,10]  = np.cos(self.spatial_state.mean()[3])*self.dt, np.sin(self.spatial_state.mean()[3])*self.dt, self.dt
        
        else:
            #TODO - sine and cosine terms for planar velocity
            self.proc_model[0,3] =  np.cos(self.spatial_state.mean()[8])*self.dt
            self.proc_model[0,4] = -np.sin(self.spatial_state.mean()[8])*self.dt
            self.proc_model[1,3] = np.sin(self.spatial_state.mean()[8])*self.dt
            self.proc_model[1,4] = np.cos(self.spatial_state.mean()[8])*self.dt
            self.proc_model[2,5] = self.dt # pz = pz + vz*dt
            self.proc_model[6,9], self.proc_model[7,10], self.proc_model[8,11]  = self.dt, self.dt, self.dt # theta = theta + omega*dt

    def compute_proc_noise(self):
        if self.obj_params['model_type'] == 'ab3dmot':
            self.proc_noise = gtsam.noiseModel.Diagonal.Sigmas(self.obj_params['proc_var'])

        elif self.obj_params['model_type'] == 'ackermann':
            self.proc_noise = gtsam.noiseModel.Diagonal.Sigmas(self.obj_params['proc_var']) # TODO compute position var as a function of vel var

        else:
            self.proc_noise = gtsam.noiseModel.Diagonal.Sigmas([0.25*self.vel_variance[0]**2*self.dt**4,
                                                                0.25*self.vel_variance[1]**2*self.dt**4,
                                                                0.25*self.vel_variance[2]**2*self.dt**4,
                                                                0.5*self.vel_variance[0]**2*self.dt**2,
                                                                0.5*self.vel_variance[1]**2*self.dt**2,
                                                                0.5*self.vel_variance[2]**2*self.dt**2,
                                                                0.25*self.rot_rate_variance[0]**2*self.dt**4,
                                                                0.25*self.rot_rate_variance[1]**2*self.dt**4,
                                                                0.25*self.rot_rate_variance[2]**2*self.dt**4,
                                                                0.5*self.rot_rate_variance[0]**2*self.dt**2,
                                                                0.5*self.rot_rate_variance[1]**2*self.dt**2,
                                                                0.5*self.rot_rate_variance[2]**2*self.dt**2,
                                                                self.size_variance[0],
                                                                self.size_variance[1],
                                                                self.size_variance[2]])
    def propagate(self, t):
        self.dt = (Time.from_msg(t) - self.timestamp).nanoseconds/10**9
        self.timestamp = Time.from_msg(t)
        self.compute_proc_model()
        self.compute_proc_noise()
        self.spatial_state = self.kf.predict(self.spatial_state,self.proc_model,np.zeros((self.n_states,self.n_states)),np.zeros((self.n_states,1)),self.proc_noise)

    def update(self, det, det_params):

        # Admin / track management
        self.timestamp = det.timestamp
        self.time_updated = det.timestamp
        self.n_missed = 0
        self.metadata = det.metadata
        self.n_matched += 1
        self.class_dist = gtsam.DiscreteDistribution(det_params['p_class_label'].likelihood(det_params[det.class_string]['idx'])*self.class_dist)

        # Compute spatial state
        rot = gtsam.Rot3(det.pose.orientation.w,det.pose.orientation.x, det.pose.orientation.y, det.pose.orientation.z)
        det_yaw = rot.rpy()[2]
        if abs(det_yaw - self.spatial_state.mean()[3])>np.pi/2:
            det_yaw - np.pi*np.sign(det_yaw - self.spatial_state.mean()[3])

        if self.obj_params['model_type'] in ['ab3dmot','ackermann']:
            self.spatial_state = self.kf.update(self.spatial_state, det_params['obs_model'], np.vstack((det.pos, det_yaw, det.size, det.class_conf)), det_params[det.class_string]['obs_var'])
        else:
            self.spatial_state = self.kf.update(self.spatial_state, det_params['obs_model'], np.vstack((det.pos, det.size)), det_params[det.class_string]['obs_var'])

        # Semantic state
        self.class_conf = det.class_conf
        hist_ind = np.searchsorted( det_params['hist_bins'], float(det.class_conf), side='right') -1
        self.track_conf = gtsam.DiscreteDistribution([gtsam.symbol('e',self.trk_id),2],[det_params['false_pos_hist'][self.class_dist.argmax()][hist_ind]*self.track_conf(0),det_params['true_pos_hist'][self.class_dist.argmax()][hist_ind]*self.track_conf(1)])
