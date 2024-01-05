from scipy.optimize import linear_sum_assignment
import numpy as np
import gtsam

def Cost(det, track, prob_class_label, det_params, tracker):

    if gtsam.DiscreteDistribution(prob_class_label.likelihood(det_params[det.class_string]['idx'])).argmax() != track.class_dist.argmax():
        class_mismatch_penalty = 20.
    else:
        class_mismatch_penalty = 0.

    # Euclidean distance between positions
    return np.linalg.norm(det.pos[:,0] - track.spatial_state.mean()[0:3]) + class_mismatch_penalty

def ComputeCostMatrix(tracker, prob_class_label, det_params):
    tracker.cost_matrix = np.zeros((len(tracker.dets),len(tracker.trks)))
    for ii,det in enumerate(tracker.dets):
        for jj,trk in enumerate(tracker.trks):
            tracker.cost_matrix[ii,jj] = Cost(det, trk, prob_class_label, det_params, tracker)
    
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

def ComputeAssignment(tracker, prob_class_label, det_params):    
    ComputeCostMatrix(tracker, prob_class_label, det_params)
    SolveCostMatrix(tracker)