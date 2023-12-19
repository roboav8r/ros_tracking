import numpy as np
import gtsam

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

    # Create sensor model and noise vector
    obs_var = gtsam.noiseModel.Diagonal.Sigmas(sensor_params['sensor_variance']) 
    obs_model = np.array(sensor_params['spatial_sensor_model']).reshape((sensor_params['spatial_meas'],sensor_params['spatial_states']))

    # Create semantic sensor model
    prob_class_label = np.array(sensor_params['semantic_sensor_model']).reshape((sensor_params['dim_classes'],sensor_params['dim_detections']))
    prob_exists_det = np.array([[sensor_params['p_false_pos'], 1 - sensor_params['p_false_pos']],[1 - sensor_params['p_missed_det'], sensor_params['p_missed_det']]])
    class_label_spec = ComputeMatrixSpec(prob_class_label)
    exists_det_spec = ComputeMatrixSpec(prob_exists_det)

    p_class_label = gtsam.DiscreteConditional((label_sym,sensor_params['dim_detections']),[(class_sym,sensor_params['dim_classes'])],class_label_spec)
    p_exists_det = gtsam.DiscreteConditional((exist_sym,2),[(det_sym,2)],exists_det_spec)

    det_idx_map = dict()
    for idx, class_name in enumerate(sensor_params['detection_classes']):
        det_idx_map[class_name] = idx

    # TODO - make unique subscription name if multiple sensors given
    tracker.subscription = tracker.create_subscription(eval(sensor_params['msg_type']),
            sensor_params['topic'],
            lambda msg: tracker.det_callback(msg, obs_model, obs_var, p_class_label, p_exists_det, det_idx_map), 
            10
    )


def CreateSensorModels(tracker):

    tracker.declare_parameter('sensors.sensor_names')
    sensor_names = tracker.get_parameter('sensors.sensor_names').get_parameter_value().string_array_value

    # Create sensor subscriber objects
    class_sym = gtsam.symbol('c',0)
    exist_sym = gtsam.symbol('e',0)
    det_sym = gtsam.symbol('d',0)

    sensor_idx = 0
    for sensor in sensor_names:
        label_sym = gtsam.symbol('l',sensor_idx)

        # Declare parameters for sensor
        tracker.declare_parameter('sensors.' + sensor + '.topic')
        tracker.declare_parameter('sensors.' + sensor + '.msg_type')
        tracker.declare_parameter('sensors.' + sensor + '.spatial_sensor_model')
        tracker.declare_parameter('sensors.' + sensor + '.spatial_states')
        tracker.declare_parameter('sensors.' + sensor + '.spatial_meas')
        tracker.declare_parameter('sensors.' + sensor + '.sensor_variance')
        tracker.declare_parameter('sensors.' + sensor + '.dim_classes')        
        tracker.declare_parameter('sensors.' + sensor + '.dim_detections')
        tracker.declare_parameter('sensors.' + sensor + '.detection_classes')
        tracker.declare_parameter('sensors.' + sensor + '.semantic_sensor_model')
        tracker.declare_parameter('sensors.' + sensor + '.p_missed_det')
        tracker.declare_parameter('sensors.' + sensor + '.p_false_pos')
        
        # Form parameter dictionary for sensor
        sensor_params = dict()
        sensor_params['name'] = sensor
        sensor_params['topic'] = tracker.get_parameter('sensors.' + sensor + '.topic').get_parameter_value().string_value
        sensor_params['msg_type'] = tracker.get_parameter('sensors.' + sensor + '.msg_type').get_parameter_value().string_value
        sensor_params['spatial_sensor_model'] = tracker.get_parameter('sensors.' + sensor + '.spatial_sensor_model').get_parameter_value().double_array_value
        sensor_params['spatial_states'] = tracker.get_parameter('sensors.' + sensor + '.spatial_states').get_parameter_value().integer_value
        sensor_params['spatial_meas'] = tracker.get_parameter('sensors.' + sensor + '.spatial_meas').get_parameter_value().integer_value
        sensor_params['sensor_variance'] = tracker.get_parameter('sensors.' + sensor + '.sensor_variance').get_parameter_value().double_array_value
        sensor_params['dim_classes'] = tracker.get_parameter('sensors.' + sensor + '.dim_classes').get_parameter_value().integer_value
        sensor_params['dim_detections'] = tracker.get_parameter('sensors.' + sensor + '.dim_detections').get_parameter_value().integer_value
        sensor_params['detection_classes'] = tracker.get_parameter('sensors.' + sensor + '.detection_classes').get_parameter_value().string_array_value
        sensor_params['semantic_sensor_model'] = tracker.get_parameter('sensors.' + sensor + '.semantic_sensor_model').get_parameter_value().double_array_value
        sensor_params['p_missed_det'] = tracker.get_parameter('sensors.' + sensor + '.p_missed_det').get_parameter_value().double_value
        sensor_params['p_false_pos'] = tracker.get_parameter('sensors.' + sensor + '.p_false_pos').get_parameter_value().double_value

        # Create sensor object
        CreateSensorModel(tracker, sensor_params, class_sym, label_sym, exist_sym, det_sym)
        sensor_idx +=1