import numpy as np

import rclpy

from tracking_msgs.msg import Track3D, Tracks3D
from foxglove_msgs.msg import SceneEntity, SceneUpdate


def ConfigureTracker(tracker):

    # Declare parameters
    tracker.declare_parameter('tracker.dim_objects', rclpy.Parameter.Type.INTEGER)
    tracker.declare_parameter('tracker.object_classes', rclpy.Parameter.Type.STRING_ARRAY)
    tracker.declare_parameter('tracker.frame_id', rclpy.Parameter.Type.STRING )
    tracker.declare_parameter('tracker.asgn_thresh', rclpy.Parameter.Type.DOUBLE )
    tracker.declare_parameter('tracker.del_thresh_list', rclpy.Parameter.Type.DOUBLE_ARRAY )
    tracker.declare_parameter('tracker.pub_thresh_list', rclpy.Parameter.Type.DOUBLE_ARRAY )
    tracker.declare_parameter('tracker.publishers.names', rclpy.Parameter.Type.STRING_ARRAY)
    tracker.declare_parameter('tracker.n_age_max_list', rclpy.Parameter.Type.INTEGER_ARRAY)
    tracker.declare_parameter('tracker.n_birth_min_list', rclpy.Parameter.Type.INTEGER_ARRAY)
    tracker.declare_parameter('tracker.trk_mgmt_method', rclpy.Parameter.Type.STRING)
    tracker.declare_parameter('object_properties.object_classes', rclpy.Parameter.Type.STRING_ARRAY)

    # Read parameters and assign to tracker object
    tracker.dim_objects = tracker.get_parameter('tracker.dim_objects').get_parameter_value().integer_value
    tracker.object_classes = tracker.get_parameter('tracker.object_classes').get_parameter_value().string_array_value
    tracker.frame_id = tracker.get_parameter('tracker.frame_id').get_parameter_value().string_value
    tracker.asgn_thresh = tracker.get_parameter('tracker.asgn_thresh').get_parameter_value().double_value
    tracker.del_thresh_list = tracker.get_parameter('tracker.del_thresh_list').get_parameter_value().double_array_value
    tracker.pub_thresh_list = tracker.get_parameter('tracker.pub_thresh_list').get_parameter_value().double_array_value
    tracker.n_age_max_list = tracker.get_parameter('tracker.n_age_max_list').get_parameter_value().integer_array_value
    tracker.n_birth_min_list= tracker.get_parameter('tracker.n_birth_min_list').get_parameter_value().integer_array_value
    tracker.trk_mgmt_method = tracker.get_parameter('tracker.trk_mgmt_method').get_parameter_value().string_value
    tracker.obj_prop_classes = tracker.get_parameter('object_properties.object_classes').get_parameter_value().string_array_value

    tracker.class_idx_map = dict()
    for idx, class_name in enumerate(tracker.object_classes):
        tracker.class_idx_map[class_name] = idx

    tracker.object_properties = dict()
    for obj_name in tracker.obj_prop_classes:
        tracker.declare_parameter('object_properties.' + obj_name + '.model_type', rclpy.Parameter.Type.STRING)
        tracker.declare_parameter('object_properties.' + obj_name + '.proc_var', rclpy.Parameter.Type.DOUBLE_ARRAY)

        tracker.object_properties[obj_name] = {'model_type': tracker.get_parameter('object_properties.' + obj_name + '.model_type').get_parameter_value().string_value, 
                                               'proc_var': tracker.get_parameter('object_properties.' + obj_name + '.proc_var').get_parameter_value().double_array_value}

    # Create publishers
    tracker.pub_names = tracker.get_parameter('tracker.publishers.names').get_parameter_value().string_array_value
    tracker.pubs = dict()
    for pub in tracker.pub_names:
        pub_dict = dict()
        tracker.declare_parameter('tracker.publishers.' + pub + '.pub_topic', rclpy.Parameter.Type.STRING)
        tracker.declare_parameter('tracker.publishers.' + pub + '.msg_type', rclpy.Parameter.Type.STRING)
        tracker.declare_parameter('tracker.publishers.' + pub + '.routine', rclpy.Parameter.Type.STRING)
        pub_dict['topic'] = tracker.get_parameter('tracker.publishers.' + pub + '.pub_topic').get_parameter_value().string_value
        pub_dict['msg_type'] = tracker.get_parameter('tracker.publishers.' + pub + '.msg_type').get_parameter_value().string_value
        pub_dict['routine'] = tracker.get_parameter('tracker.publishers.' + pub + '.routine').get_parameter_value().string_value
    
        tracker.pubs[pub] = (pub_dict)
        exec('tracker.%s  = tracker.create_publisher(%s,\'%s\',10)' % (pub, pub_dict['msg_type'], pub_dict['topic']))