from rclpy.time import Time
from ros_tracking.datatypes import GraphTrack

# Helper functions related to track creation and deletion

def CreateTrack(tracker, det, prob_class_det, det_idx_map):
    # TODO - don't create track for void/ignored objects or false detections
    # return tracker.object_classes[det_idx_map[det.class_string]] not in ['false_detection','void_ignore']
    return True

def ValidTrack(trk, tracker):
    tracker.object_classes[trk.class_dist.argmax()] not in ['false_detection','void_ignore']
    print(trk.class_dist.argmax())

    return (tracker.object_classes[trk.class_dist.argmax()] not in ['false_detection','void_ignore']) and ((trk.missed_det < tracker.trk_delete_missed_det) or ((tracker.get_clock().now() - trk.timestamp).nanoseconds/10**9 < tracker.trk_delete_timeout))

def CreateTracks(tracker, prob_class_det, det_idx_map):
    while tracker.dets:
        if (len(tracker.dets)-1) in tracker.det_asgn_idx: # If detection at end of list is matched, remove it
            tracker.dets.pop()

        elif CreateTrack(tracker, tracker.dets[-1], prob_class_det, det_idx_map): # Check to see if track should be created from detection
            tracker.trks.append(GraphTrack(tracker.trk_id_count,tracker.dets.pop(), prob_class_det, det_idx_map))
            tracker.trk_id_count += 1

        else: # Otherwise, remove the detection
            tracker.dets.pop()

def DeleteTracks(tracker):
    tracker.trks = [track for track in tracker.trks if ValidTrack(track, tracker)]
    # tracker.tracks = [track for track in tracker.trks if track.class_conf > tracker.trk_delete_thresh]
    # tracker.tracks = [track for track in tracker.trks if track.missed_det < tracker.trk_delete_missed_det]
    # tracker.tracks = [track for track in tracker.trks if ((rclpy.Time.now() - track.timestamp).to_sec() < tracker.trk_delete_time)]