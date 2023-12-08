import numpy as np

def ConfigureTracker(tracker):

    # Declare parameters
    tracker.declare_parameter('tracker.dim_objects')
    tracker.declare_parameter('tracker.object_classes')
    tracker.declare_parameter('tracker.frame_id')
    
    tracker.declare_parameter('tracker.trk_pub_topic')
    tracker.declare_parameter('tracker.trk_msg_type')
    tracker.declare_parameter('tracker.scene_pub_topic')
    tracker.declare_parameter('tracker.scene_msg_type')
    
    tracker.declare_parameter('tracker.asgn_thresh')
    tracker.declare_parameter('tracker.trk_timeout')

    # Read parameters and assign to tracker object
    tracker.dim_objects = tracker.get_parameter('tracker.dim_objects').get_parameter_value().integer_value
    tracker.object_classes = tracker.get_parameter('tracker.object_classes').get_parameter_value().string_array_value
    tracker.frame_id = tracker.get_parameter('tracker.frame_id').get_parameter_value().string_value

    tracker.trk_pub_topic = tracker.get_parameter('tracker.trk_pub_topic').get_parameter_value().string_value
    tracker.trk_msg_type = tracker.get_parameter('tracker.trk_msg_type').get_parameter_value().string_value
    tracker.scene_pub_topic = tracker.get_parameter('tracker.scene_pub_topic').get_parameter_value().string_value
    tracker.scene_msg_type = tracker.get_parameter('tracker.scene_msg_type').get_parameter_value().string_value

    tracker.asgn_thresh = tracker.get_parameter('tracker.asgn_thresh').get_parameter_value().double_value
    tracker.trk_timeout = tracker.get_parameter('tracker.trk_timeout').get_parameter_value().double_value

    tracker.class_idx_map = dict()
    for idx, class_name in enumerate(tracker.object_classes):
        tracker.class_idx_map[class_name] = idx