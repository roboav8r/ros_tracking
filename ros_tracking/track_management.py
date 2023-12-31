import gtsam

from rclpy.time import Time
from ros_tracking.datatypes import GraphTrack

# Helper functions related to track creation and deletion

def CreateTrack(tracker, det, det_params):
    # return tracker.object_classes[gtsam.DiscreteDistribution(prob_class_det.likelihood(det_idx_map[det.class_string])).argmax()] not in ['false_detection','void_ignore']
    # return float(det.class_conf) > 0.3
    return True

def ValidTrack(trk, tracker):
    # return (tracker.object_classes[trk.class_dist.argmax()] not in ['false_detection','void_ignore']) and ((trk.missed_det < tracker.trk_delete_missed_det) or ((tracker.get_clock().now() - trk.timestamp).nanoseconds/10**9 < tracker.trk_timeout))
    # return (tracker.object_classes[trk.class_dist.argmax()] not in ['false_detection','void_ignore']) and (((tracker.get_clock().now() - trk.timestamp).nanoseconds/10**9 < tracker.trk_timeout))
    # return (tracker.object_classes[trk.class_dist.argmax()] not in ['false_detection','void_ignore'])
    
    if tracker.trk_mgmt_method=="count":
        return trk.n_missed <= tracker.n_age_max_list[trk.class_dist.argmax()]
    
    if tracker.trk_mgmt_method=="prob":
        return trk.track_conf(1) > tracker.del_thresh_list[trk.class_dist.argmax()]

def CreateTracks(tracker, det_params):
    while tracker.dets:
        if (len(tracker.dets)-1) in tracker.det_asgn_idx: # If detection at end of list is matched, remove it
            tracker.dets.pop()

        elif CreateTrack(tracker, tracker.dets[-1], det_params): # Check to see if track should be created from detection
            tracker.trks.append(GraphTrack(tracker,tracker.dets.pop(), det_params))
            tracker.trk_id_count += 1

        else: # Otherwise, remove the detection
            tracker.dets.pop()

def DeleteTracks(tracker):
    tracker.trks = [track for track in tracker.trks if ValidTrack(track, tracker)]