import numpy as np
import gtsam

from tracking_msgs.msg import Tracks3D, Track3D, Detections3D, Detection3D

def CreateSensorModel(tracker,sensor_params):

    # Create sensor model and noise vector
    # obs_var = np.array(sensor_params['sensor_variance'])
    obs_var = gtsam.noiseModel.Diagonal.Sigmas(sensor_params['sensor_variance']) 
    obs_model = np.array(sensor_params['sensor_model']).reshape((sensor_params['dim_meas'],sensor_params['dim_states']))
    # print(sensor_var)
    # print(sensor_model)

    # TODO - make more subscriptions for multiple sensors
    tracker.subscription = tracker.create_subscription(eval(sensor_params['msg_type']),
            sensor_params['topic'],
            lambda msg: tracker.det_callback(msg, obs_model, obs_var), 10
    )

def CreateSensorModels(tracker):

    tracker.declare_parameter('sensors.sensor_names')
    sensor_names = tracker.get_parameter('sensors.sensor_names').get_parameter_value().string_array_value
    print("Got sensor names")
    print(sensor_names)

    # Create sensor subscriber objects
    for sensor in sensor_names:

        # Declare parameters for sensor
        tracker.declare_parameter('sensors.' + sensor + '.topic')
        tracker.declare_parameter('sensors.' + sensor + '.msg_type')
        tracker.declare_parameter('sensors.' + sensor + '.sensor_model')
        tracker.declare_parameter('sensors.' + sensor + '.dim_states')
        tracker.declare_parameter('sensors.' + sensor + '.dim_meas')
        tracker.declare_parameter('sensors.' + sensor + '.sensor_variance')
        
        # Form parameter dictionary for sensor
        sensor_params = dict()
        sensor_params['name'] = sensor
        sensor_params['topic'] = tracker.get_parameter('sensors.' + sensor + '.topic').get_parameter_value().string_value
        sensor_params['msg_type'] = tracker.get_parameter('sensors.' + sensor + '.msg_type').get_parameter_value().string_value
        sensor_params['sensor_model'] = tracker.get_parameter('sensors.' + sensor + '.sensor_model').get_parameter_value().double_array_value
        sensor_params['dim_states'] = tracker.get_parameter('sensors.' + sensor + '.dim_states').get_parameter_value().integer_value
        sensor_params['dim_meas'] = tracker.get_parameter('sensors.' + sensor + '.dim_meas').get_parameter_value().integer_value
        sensor_params['sensor_variance'] = tracker.get_parameter('sensors.' + sensor + '.sensor_variance').get_parameter_value().double_array_value
        
        # Create sensor object
        CreateSensorModel(tracker, sensor_params)