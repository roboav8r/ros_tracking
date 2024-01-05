import gtsam

from rclpy.time import Time
from ros_tracking.datatypes import GraphTrack

# Helper functions related to track creation and deletion
def CreateTrack(tracker, det, det_params):
    return True

def ValidTrack(trk, tracker):
    
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