from scipy.optimize import linear_sum_assignment
import numpy as np

def Cost(det, track):
    # Euclidean distance between positions
    return np.linalg.norm(det.pos[:,0] - track.state.mean()[0:3])

def ComputeCostMatrix(tracker):
    tracker.cost_matrix = np.zeros((len(tracker.dets),len(tracker.trks)))
    for ii,det in enumerate(tracker.dets):
        for jj,trk in enumerate(tracker.trks):
            tracker.cost_matrix[ii,jj] = Cost(det,trk)
    
def SolveCostMatrix(tracker):
    tracker.det_asgn_idx, tracker.trk_asgn_idx = linear_sum_assignment(tracker.cost_matrix)
    tracker.det_asgn_idx, tracker.trk_asgn_idx = list(tracker.det_asgn_idx), list(tracker.trk_asgn_idx)
    
    # If cost above threshold, remove the match from assignment vector
    assert(len(tracker.det_asgn_idx) == len(tracker.trk_asgn_idx))
    ii = len(tracker.det_asgn_idx)
    while ii:
        idx = ii-1
        if tracker.cost_matrix[tracker.det_asgn_idx[idx],tracker.trk_asgn_idx[idx]] > tracker.asgn_thresh:
            del tracker.det_asgn_idx[idx], tracker.trk_asgn_idx[idx]       
        ii -=1
    assert(len(tracker.det_asgn_idx) == len(tracker.trk_asgn_idx))

def ComputeAssignment(tracker):    
    ComputeCostMatrix(tracker)
    SolveCostMatrix(tracker)