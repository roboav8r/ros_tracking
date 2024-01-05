import numpy as np
import gtsam

import rclpy

from tracking_msgs.msg import Detections3D

def ComputeMatrixSpec(array):
    spec = ''
    for row in range(array.shape[0]):
        if row!=0:
            spec+=' '    
        for col in range(array.shape[1]):
            if col!=0:
                spec+='/'
            spec+=str(array[row,col])
    
    return spec

def CreateSensorModel(tracker, sensor_params, class_sym, label_sym, exist_sym, det_sym):
    det_params = dict()

    # Create sensor model and noise vector
    det_params['pos_var'] = np.array(sensor_params['pos_variance']).reshape((sensor_params['dim_detections'],3))
    det_params['yaw_var'] = np.array(sensor_params['yaw_variance']).reshape((sensor_params['dim_detections'],1))
    det_params['size_var'] = np.array(sensor_params['size_variance']).reshape((sensor_params['dim_detections'],3))
    det_params['conf_var'] = np.array(sensor_params['conf_variance']).reshape((sensor_params['dim_detections'],1))
    det_params['obs_model'] = np.array(sensor_params['spatial_sensor_model']).reshape((sensor_params['spatial_meas'],sensor_params['spatial_states']))

    # Create semantic sensor model
    prob_class_label = np.array(sensor_params['semantic_sensor_model']).reshape((sensor_params['dim_classes'],sensor_params['dim_detections']))
    class_label_spec = ComputeMatrixSpec(prob_class_label)
    det_params['p_class_label'] = gtsam.DiscreteConditional((label_sym,sensor_params['dim_detections']),[(class_sym,sensor_params['dim_classes'])],class_label_spec)

    det_params['class_sym'] = class_sym
    det_params['label_sym'] = label_sym
    det_params['exist_sym'] = exist_sym
    det_params['det_sym'] = det_sym

    det_params['hist_bins'] = sensor_params['hist_bins']
    det_params['false_pos_hist'] = np.array(sensor_params['false_pos_hist']).reshape((sensor_params['dim_classes'],len(det_params['hist_bins']) - 1))
    det_params['true_pos_hist'] = np.array(sensor_params['true_pos_hist']).reshape((sensor_params['dim_classes'],len(det_params['hist_bins']) - 1))
    det_params['p_missed_det'] = sensor_params['p_missed_det_list']

    for idx, class_name in enumerate(sensor_params['detection_classes']):
        det_params[class_name] = dict()
        det_params[class_name]['idx'] = idx
        det_params[class_name]['obs_var'] = gtsam.noiseModel.Diagonal.Variances(np.concatenate((det_params['pos_var'][idx,:], det_params['yaw_var'][idx,:], det_params['size_var'][idx,:], det_params['conf_var'][idx,:])))

    tracker.subscription = tracker.create_subscription(eval(sensor_params['msg_type']),
            sensor_params['topic'],
            lambda msg: tracker.det_callback(msg, det_params), 
            10
    )

def ReconfigureSensorModel(tracker, sensor_params, class_sym, label_sym, exist_sym, det_sym):
    det_params = dict()

    # Create sensor model and noise vector
    det_params['pos_var'] = np.array(sensor_params['pos_variance']).reshape((sensor_params['dim_detections'],3))
    det_params['yaw_var'] = np.array(sensor_params['yaw_variance']).reshape((sensor_params['dim_detections'],1))
    det_params['size_var'] = np.array(sensor_params['size_variance']).reshape((sensor_params['dim_detections'],3))
    det_params['conf_var'] = np.array(sensor_params['conf_variance']).reshape((sensor_params['dim_detections'],1))
    det_params['obs_model'] = np.array(sensor_params['spatial_sensor_model']).reshape((sensor_params['spatial_meas'],sensor_params['spatial_states']))

    # Create semantic sensor model
    prob_class_label = np.array(sensor_params['semantic_sensor_model']).reshape((sensor_params['dim_classes'],sensor_params['dim_detections']))
    class_label_spec = ComputeMatrixSpec(prob_class_label)
    det_params['p_class_label'] = gtsam.DiscreteConditional((label_sym,sensor_params['dim_detections']),[(class_sym,sensor_params['dim_classes'])],class_label_spec)

    det_params['class_sym'] = class_sym
    det_params['label_sym'] = label_sym
    det_params['exist_sym'] = exist_sym
    det_params['det_sym'] = det_sym

    det_params['hist_bins'] = sensor_params['hist_bins']
    det_params['false_pos_hist'] = np.array(sensor_params['false_pos_hist']).reshape((sensor_params['dim_classes'],len(det_params['hist_bins']) - 1))
    det_params['true_pos_hist'] = np.array(sensor_params['true_pos_hist']).reshape((sensor_params['dim_classes'],len(det_params['hist_bins']) - 1))
    det_params['p_missed_det'] = sensor_params['p_missed_det_list']

    for idx, class_name in enumerate(sensor_params['detection_classes']):
        det_params[class_name] = dict()
        det_params[class_name]['idx'] = idx
        det_params[class_name]['obs_var'] = gtsam.noiseModel.Diagonal.Variances(np.concatenate((det_params['pos_var'][idx,:], det_params['yaw_var'][idx,:], det_params['size_var'][idx,:], det_params['conf_var'][idx,:])))


def CreateSensorModels(tracker):

    tracker.declare_parameter('sensors.sensor_names', rclpy.Parameter.Type.STRING_ARRAY)
    sensor_names = tracker.get_parameter('sensors.sensor_names').get_parameter_value().string_array_value

    # Create sensor subscriber objects
    class_sym = gtsam.symbol('c',0)
    exist_sym = gtsam.symbol('e',0)
    det_sym = gtsam.symbol('d',0)

    sensor_idx = 0
    for sensor in sensor_names:
        label_sym = gtsam.symbol('l',sensor_idx)

        # Declare parameters for sensor
        tracker.declare_parameter('sensors.' + sensor + '.topic', rclpy.Parameter.Type.STRING)
        tracker.declare_parameter('sensors.' + sensor + '.msg_type', rclpy.Parameter.Type.STRING)
        tracker.declare_parameter('sensors.' + sensor + '.spatial_sensor_model',rclpy.Parameter.Type.DOUBLE_ARRAY)
        tracker.declare_parameter('sensors.' + sensor + '.spatial_states')
        tracker.declare_parameter('sensors.' + sensor + '.spatial_meas')
        tracker.declare_parameter('sensors.' + sensor + '.sensor_variance.pos', rclpy.Parameter.Type.DOUBLE_ARRAY)
        tracker.declare_parameter('sensors.' + sensor + '.sensor_variance.size', rclpy.Parameter.Type.DOUBLE_ARRAY)
        tracker.declare_parameter('sensors.' + sensor + '.sensor_variance.yaw', rclpy.Parameter.Type.DOUBLE_ARRAY)
        tracker.declare_parameter('sensors.' + sensor + '.sensor_variance.conf', rclpy.Parameter.Type.DOUBLE_ARRAY)
        tracker.declare_parameter('sensors.' + sensor + '.dim_classes')        
        tracker.declare_parameter('sensors.' + sensor + '.dim_detections')
        tracker.declare_parameter('sensors.' + sensor + '.detection_classes')
        tracker.declare_parameter('sensors.' + sensor + '.semantic_sensor_model', rclpy.Parameter.Type.DOUBLE_ARRAY)
        tracker.declare_parameter('sensors.' + sensor + '.p_missed_det_list')
        tracker.declare_parameter('sensors.' + sensor + '.hist_bins', rclpy.Parameter.Type.DOUBLE_ARRAY)
        tracker.declare_parameter('sensors.' + sensor + '.false_pos_hist', rclpy.Parameter.Type.DOUBLE_ARRAY)
        tracker.declare_parameter('sensors.' + sensor + '.true_pos_hist', rclpy.Parameter.Type.DOUBLE_ARRAY)

        # Form parameter dictionary for sensor
        sensor_params = dict()
        sensor_params['name'] = sensor
        sensor_params['topic'] = tracker.get_parameter('sensors.' + sensor + '.topic').get_parameter_value().string_value
        sensor_params['msg_type'] = tracker.get_parameter('sensors.' + sensor + '.msg_type').get_parameter_value().string_value
        sensor_params['spatial_sensor_model'] = tracker.get_parameter('sensors.' + sensor + '.spatial_sensor_model').get_parameter_value().double_array_value
        sensor_params['spatial_states'] = tracker.get_parameter('sensors.' + sensor + '.spatial_states').get_parameter_value().integer_value
        sensor_params['spatial_meas'] = tracker.get_parameter('sensors.' + sensor + '.spatial_meas').get_parameter_value().integer_value
        sensor_params['pos_variance'] = tracker.get_parameter('sensors.' + sensor + '.sensor_variance.pos').get_parameter_value().double_array_value
        sensor_params['size_variance'] = tracker.get_parameter('sensors.' + sensor + '.sensor_variance.size').get_parameter_value().double_array_value
        sensor_params['yaw_variance'] = tracker.get_parameter('sensors.' + sensor + '.sensor_variance.yaw').get_parameter_value().double_array_value
        sensor_params['conf_variance'] = tracker.get_parameter('sensors.' + sensor + '.sensor_variance.conf').get_parameter_value().double_array_value
        sensor_params['dim_classes'] = tracker.get_parameter('sensors.' + sensor + '.dim_classes').get_parameter_value().integer_value
        sensor_params['dim_detections'] = tracker.get_parameter('sensors.' + sensor + '.dim_detections').get_parameter_value().integer_value
        sensor_params['detection_classes'] = tracker.get_parameter('sensors.' + sensor + '.detection_classes').get_parameter_value().string_array_value
        sensor_params['semantic_sensor_model'] = tracker.get_parameter('sensors.' + sensor + '.semantic_sensor_model').get_parameter_value().double_array_value
        sensor_params['p_missed_det_list'] = tracker.get_parameter('sensors.' + sensor + '.p_missed_det_list').get_parameter_value().double_array_value

        sensor_params['hist_bins'] = tracker.get_parameter('sensors.' + sensor + '.hist_bins').get_parameter_value().double_array_value
        sensor_params['false_pos_hist'] = tracker.get_parameter('sensors.' + sensor + '.false_pos_hist').get_parameter_value().double_array_value
        sensor_params['true_pos_hist'] = tracker.get_parameter('sensors.' + sensor + '.true_pos_hist').get_parameter_value().double_array_value

        # Create sensor object
        CreateSensorModel(tracker, sensor_params, class_sym, label_sym, exist_sym, det_sym)
        sensor_idx +=1

def ReconfigureSensorModels(tracker):

    sensor_names = tracker.get_parameter('sensors.sensor_names').get_parameter_value().string_array_value

    # Create sensor subscriber objects
    class_sym = gtsam.symbol('c',0)
    exist_sym = gtsam.symbol('e',0)
    det_sym = gtsam.symbol('d',0)

    sensor_idx = 0
    for sensor in sensor_names:
        label_sym = gtsam.symbol('l',sensor_idx)

        # Form parameter dictionary for sensor
        sensor_params = dict()
        sensor_params['name'] = sensor
        sensor_params['topic'] = tracker.get_parameter('sensors.' + sensor + '.topic').get_parameter_value().string_value
        sensor_params['msg_type'] = tracker.get_parameter('sensors.' + sensor + '.msg_type').get_parameter_value().string_value
        sensor_params['spatial_sensor_model'] = tracker.get_parameter('sensors.' + sensor + '.spatial_sensor_model').get_parameter_value().double_array_value
        sensor_params['spatial_states'] = tracker.get_parameter('sensors.' + sensor + '.spatial_states').get_parameter_value().integer_value
        sensor_params['spatial_meas'] = tracker.get_parameter('sensors.' + sensor + '.spatial_meas').get_parameter_value().integer_value
        sensor_params['pos_variance'] = tracker.get_parameter('sensors.' + sensor + '.sensor_variance.pos').get_parameter_value().double_array_value
        sensor_params['size_variance'] = tracker.get_parameter('sensors.' + sensor + '.sensor_variance.size').get_parameter_value().double_array_value
        sensor_params['yaw_variance'] = tracker.get_parameter('sensors.' + sensor + '.sensor_variance.yaw').get_parameter_value().double_array_value
        sensor_params['conf_variance'] = tracker.get_parameter('sensors.' + sensor + '.sensor_variance.conf').get_parameter_value().double_array_value
        sensor_params['dim_classes'] = tracker.get_parameter('sensors.' + sensor + '.dim_classes').get_parameter_value().integer_value
        sensor_params['dim_detections'] = tracker.get_parameter('sensors.' + sensor + '.dim_detections').get_parameter_value().integer_value
        sensor_params['detection_classes'] = tracker.get_parameter('sensors.' + sensor + '.detection_classes').get_parameter_value().string_array_value
        sensor_params['semantic_sensor_model'] = tracker.get_parameter('sensors.' + sensor + '.semantic_sensor_model').get_parameter_value().double_array_value
        sensor_params['p_missed_det_list'] = tracker.get_parameter('sensors.' + sensor + '.p_missed_det_list').get_parameter_value().double_array_value

        sensor_params['hist_bins'] = tracker.get_parameter('sensors.' + sensor + '.hist_bins').get_parameter_value().double_array_value
        sensor_params['false_pos_hist'] = tracker.get_parameter('sensors.' + sensor + '.false_pos_hist').get_parameter_value().double_array_value
        sensor_params['true_pos_hist'] = tracker.get_parameter('sensors.' + sensor + '.true_pos_hist').get_parameter_value().double_array_value

        # Create sensor object
        ReconfigureSensorModel(tracker, sensor_params, class_sym, label_sym, exist_sym, det_sym)
        sensor_idx +=1