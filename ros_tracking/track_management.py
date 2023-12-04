from ros_tracking.datatypes import GraphTrack

# Helper functions related to track creation and deletion

def CreateTrack(det):
    return True

# def create_tracks(tracker):
#     # Handle unmatched detections / initialize new tracks
#     while tracker.detections:
#         if (len(tracker.detections)-1) in tracker.det_asgn_idx: # If detection at end of array is matched
#             tracker.detections.pop() # Remove it
#         else: 
#             tracker.tracks.append(GraphTrack(tracker.detections.pop(),sensor_mdl,tracker.trk_id_count)) # Otherwise create a new track
#             tracker.trk_id_count += 1

# def delete_tracks(tracker):
#     tracker.tracks = [track for track in tracker.tracks if track.class_conf > tracker.trk_delete_thresh]
#     tracker.tracks = [track for track in tracker.tracks if track.missed_det < tracker.trk_delete_missed_det]
#     tracker.tracks = [track for track in tracker.tracks if ((rospy.Time.now() - track.timestamp).to_sec() < tracker.trk_delete_time)]