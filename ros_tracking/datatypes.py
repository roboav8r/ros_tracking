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
        # TODO - id? sym?
       
        # Spatial properties
        # TODO - gtsam pose, vector types
        self.pose = det_msg.pose
        self.pos = np.array([[det_msg.pose.position.x], [det_msg.pose.position.y], [det_msg.pose.position.z]])
        self.size = np.array([[det_msg.bbox.size.x], [det_msg.bbox.size.y], [det_msg.bbox.size.z]])

        # Semantic Properties
        # TODO - GTSAM distribution
        # self.class_id 
        self.metadata = det_msg.metadata
        self.class_string = det_msg.class_string
        self.class_conf = det_msg.class_confidence
        # self.attribute = det_msg.attribute


class GraphTrack():
    def __init__(self, id, graph_det, prob_class_label, det_params):
        # Admin
        self.trk_id = id
        self.timestamp = graph_det.timestamp
        self.time_created = graph_det.timestamp
        self.time_updated = graph_det.timestamp
        self.dt = 0.
        self.metadata = graph_det.metadata
        self.n_missed = 0
        self.n_matched = 1

        # Get semantic parameters
        self.class_dist = gtsam.DiscreteDistribution(prob_class_label.likelihood(det_params[graph_det.class_string]['idx']))
        self.class_conf = graph_det.class_conf

        # Compute existence probability
        hist_ind = np.searchsorted( det_params['hist_bins'], float(graph_det.class_conf), side='right') -1
        # p_exists_det_matrix = np.array([[det_params['false_pos_hist'][hist_ind], det_params['true_pos_hist'][hist_ind]],
        #                              [1 - det_params['p_missed_det'][self.class_dist.argmax()], det_params['p_missed_det'][self.class_dist.argmax()]]])
        # exists_det_spec = ComputeMatrixSpec(p_exists_det_matrix)
        # self.p_exists_det = gtsam.DiscreteConditional((det_params['exist_sym'],2),[(det_params['det_sym'],2)],exists_det_spec)

        # self.track_conf = gtsam.DiscreteDistribution(self.p_exists_det.likelihood(0))
        self.track_conf = gtsam.DiscreteDistribution([gtsam.symbol('e',self.trk_id),2],[det_params['false_pos_hist'][hist_ind],det_params['true_pos_hist'][hist_ind]])

        # print("INIT TRACK CONF")
        # print("Det conf is %s" % graph_det.class_conf)
        # print("conditional prob: %s" % self.p_exists_det)
        # print("track conf: %s" % self.track_conf)

        # self.track_conf = gtsam.DiscreteDistribution([gtsam.symbol('t',self.trk_id),2],[det_params['false_pos_hist'][hist_ind],det_params['true_pos_hist'][hist_ind]])

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
        self.vel_variance = np.array([[0.5],[0.1],[0.1]]) # In tracker frame TODO - get this from object, attribute
        self.rot_rate_variance = np.array([[0.05],[0.05],[0.25]]) # In tracker frame TODO - get this from object, attribute
        self.size_variance = np.array([[0.005],[0.005],[0.005]])
        self.proc_model = np.diag(np.ones(15))
        self.proc_noise = gtsam.noiseModel.Diagonal.Sigmas([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1])

    def compute_proc_model(self):
        #TODO - sine and cosine terms for planar velocity
        self.proc_model[0,3] =  np.cos(self.spatial_state.mean()[8])*self.dt
        self.proc_model[0,4] = -np.sin(self.spatial_state.mean()[8])*self.dt
        self.proc_model[1,3] = np.sin(self.spatial_state.mean()[8])*self.dt
        self.proc_model[1,4] = np.cos(self.spatial_state.mean()[8])*self.dt
        self.proc_model[2,5] = self.dt # pz = pz + vz*dt
        self.proc_model[6,9], self.proc_model[7,10], self.proc_model[8,11]  = self.dt, self.dt, self.dt # theta = theta + omega*dt

    def compute_proc_noise(self):
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
        self.spatial_state = self.kf.predict(self.spatial_state,self.proc_model,np.zeros((15,15)),np.zeros((15,1)),self.proc_noise)

    def update(self, det, obs_model, obs_noise, prob_class_label, det_params):
        self.spatial_state = self.kf.update(self.spatial_state, obs_model, np.vstack((det.pos, det.size)), obs_noise)
        
        # # TODO make helper function for rpy->quat
        # cr = np.cos(0.5*self.spatial_state.mean()[6])
        # sr = np.sin(0.5*self.spatial_state.mean()[6])
        # cp = np.cos(0.5*self.spatial_state.mean()[7])
        # sp = np.sin(0.5*self.spatial_state.mean()[7])
        # cy = np.cos(0.5*self.spatial_state.mean()[8])
        # sy = np.sin(0.5*self.spatial_state.mean()[8])

        # self.orientation = gtsam.Rot3(cr*cp*cy + sr*sp*sy,
        #                               sr*cp*cy - cr*sp*sy,
        #                               cr*sp*cy + sr*cp*sy,
        #                               cr*cp*sy - sr*sp*cy)
        self.timestamp = det.timestamp
        self.time_updated = det.timestamp
        self.n_missed = 0
        self.metadata = det.metadata
        self.n_matched += 1
        self.class_dist = gtsam.DiscreteDistribution(prob_class_label.likelihood(det_params[det.class_string]['idx'])*self.class_dist)


        # Compute existence probability

        hist_ind = np.searchsorted( det_params['hist_bins'], float(det.class_conf), side='right') -1
        # print("UPDATE")
        # print("Track conf was %s, got det conf %s" % (self.track_conf, det.class_conf))
        # print("False pos is %s, true pos is %s" % (det_params['false_pos_hist'][hist_ind],det_params['true_pos_hist'][hist_ind]))
        
        # p_exists_det_matrix = np.array([[det_params['false_pos_hist'][hist_ind], det_params['true_pos_hist'][hist_ind]],
        #                              [1 - det_params['p_missed_det'][self.class_dist.argmax()], det_params['p_missed_det'][self.class_dist.argmax()]]])
        # exists_det_spec = ComputeMatrixSpec(p_exists_det_matrix)
        # self.p_exists_det = gtsam.DiscreteConditional((det_params['exist_sym'],2),[(det_params['det_sym'],2)],exists_det_spec)

        # self.track_conf = gtsam.DiscreteDistribution(self.p_exists_det.likelihood(0))
        self.track_conf = gtsam.DiscreteDistribution([gtsam.symbol('e',self.trk_id),2],[det_params['false_pos_hist'][hist_ind]*self.track_conf(0),det_params['true_pos_hist'][hist_ind]*self.track_conf(1)])
        # print("Now it's %s" % (self.track_conf))



        # p_exists_det_matrix = np.array([[det_params['p_false_pos'][det_params[det.class_string]['idx']], 1 - det_params['p_false_pos'][det_params[det.class_string]['idx']]],
        #                              [1 - det_params['p_missed_det'][self.class_dist.argmax()], det_params['p_missed_det'][self.class_dist.argmax()]]])
        # exists_det_spec = ComputeMatrixSpec(p_exists_det_matrix)
        # self.p_exists_det = gtsam.DiscreteConditional((det_params['exist_sym'],2),[(det_params['det_sym'],2)],exists_det_spec)

        # self.track_conf = gtsam.DiscreteDistribution(prob_exists_det.likelihood(0))
        # 


        # hist_ind = np.searchsorted( det_params['hist_bins'], float(det.class_conf), side='right') -1

        # self.track_conf = gtsam.DiscreteDistribution(gtsam.DiscreteDistribution([gtsam.symbol('t',self.trk_id),2],[det_params['false_pos_hist'][hist_ind],det_params['true_pos_hist'][hist_ind]])*self.track_conf)

        # self.class_conf = det.class_conf



        # self.track_conf = gtsam.DiscreteDistribution(prob_exists_det.likelihood(0)*self.track_conf)
        # if det.size is not [0,0,0]: # Only update box size if it's available
        #     self.obj_depth, self.obj_width, self.obj_height = det.obj_depth, det.obj_width, det.obj_height
        # self.transform = det.trk_transform