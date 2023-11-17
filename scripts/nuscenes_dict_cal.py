# TO DO
# - move factors to separate file and consolidate
# - move parameters to yaml, allow command line inputs
# - add multiple files to same graph
# - investigate blacklist items

import json
import gtsam
import numpy as np
from typing import List, Optional

# Constants
filepath = '/home/jd/tracking_ws/src/ros_tracking/data/dict/NuScenes-v1.0-mini-scene-0061-megvii.json'
objects = json.load(open(filepath))
blacklist = ['ac26']
motion_models = {
    'human.pedestrian.adult': {
        'pedestrian.moving': 'dynamic'
    },
    'human.pedestrian.construction_worker': {
        'pedestrian.moving': 'dynamic'
    },
    'movable_object.trafficcone': {'default': 'static'},
    'movable_object.barrier': {'default': 'static'},
    'movable_object.pushable_pullable': {'default': 'static'},
    'movable_object.debris': {'default': 'static'},
}

# Custom factors
def const_pos_error(constants, this: gtsam.CustomFactor,
               values: gtsam.Values,
               jacobians: Optional[List[np.ndarray]]) -> np.ndarray:

    # Get position annotation
    state = constants[0]
    p = gtsam.Point3(state['pos'].x,state['pos'].y,state['pos'].z)
    
    # Get variable values
    pos_key = this.keys()[0]
    pos = values.atVector(pos_key)
        
    # Compute error
    error = pos - p
    
    if jacobians is not None:
        jacobians[0] = np.eye(3)

    return error

def vel_dyn_const_error(constants, this: gtsam.CustomFactor,
               values: gtsam.Values,
               jacobians: Optional[List[np.ndarray]]) -> np.ndarray:

    last_state = constants[0]
    state = constants[1]

    # Extract constants from state
    last_p = gtsam.Point3(last_state['pos'].x,last_state['pos'].y,last_state['pos'].z)
    p = gtsam.Point3(state['pos'].x,state['pos'].y,state['pos'].z)
    dt = (state['stamp'].seconds + state['stamp'].nanos/10**9) - (last_state['stamp'].seconds + last_state['stamp'].nanos/10**9)
    
    R = gtsam.Rot3(state['rot'].w,state['rot'].x,state['rot'].y,state['rot'].z)
    last_R = gtsam.Rot3(last_state['rot'].w,last_state['rot'].x,last_state['rot'].y,last_state['rot'].z)    
    
    # Get variable values
    last_vel_key = this.keys()[0]
    vel_key = this.keys()[1]
    last_vel, vel = values.atVector(last_vel_key), values.atVector(vel_key)
        
    # Compute error
    error = (p - (last_p + dt*(last_R.rotate(last_vel) + R.rotate(vel))/2)) 
    
    if jacobians is not None:
        jacobians[0] = -last_R.matrix()*dt/2 # last vel
        jacobians[1] = -R.matrix()*dt/2 # vel
    return error

def vel_eq_error(this: gtsam.CustomFactor,
               values: gtsam.Values,
               jacobians: Optional[List[np.ndarray]]) -> np.ndarray:

    # Get variable values
    last_vel_key = this.keys()[0]
    vel_key = this.keys()[1]
    last_vel, vel = values.atVector(last_vel_key), values.atVector(vel_key)
        
    # Compute error
    error = last_vel - vel
    
    if jacobians is not None:
        jacobians[0] = np.eye(3) # last vel
        jacobians[1] = -np.eye(3) # vel

    return error

def acc_dyn_const_error(constants, this: gtsam.CustomFactor,
               values: gtsam.Values,
               jacobians: Optional[List[np.ndarray]]) -> np.ndarray:

    last_state = constants[0]
    state = constants[1]

    # Extract constants from state
    dt = (state['stamp'].seconds + state['stamp'].nanos/10**9) - (last_state['stamp'].seconds + last_state['stamp'].nanos/10**9)
    
    # Get variable values
    last_vel_key = this.keys()[0]
    vel_key = this.keys()[1]
    last_vel, vel = values.atVector(last_vel_key), values.atVector(vel_key)
    
    last_acc_key = this.keys()[2]
    acc_key = this.keys()[3]
    last_acc, acc = values.atVector(last_acc_key), values.atVector(acc_key)
        
    # Compute error
    error = (vel - (last_vel + dt*(last_acc + acc)/2)) 
    
    if jacobians is not None:
        jacobians[0] = -np.eye(3) # last vel
        jacobians[1] = np.eye(3) # vel
        jacobians[2] = -np.eye(3)*dt/2 # last acc
        jacobians[3] = -np.eye(3)*dt/2 # acc

    return error

def acc_eq_error(this: gtsam.CustomFactor,
               values: gtsam.Values,
               jacobians: Optional[List[np.ndarray]]) -> np.ndarray:

    # Get variable values
    last_acc_key = this.keys()[0]
    acc_key = this.keys()[1]
    last_acc, acc = values.atVector(last_acc_key), values.atVector(acc_key)
        
    # Compute error
    error = last_acc - acc
    
    if jacobians is not None:
        jacobians[0] = np.eye(3) # last acc
        jacobians[1] = -np.eye(3) # acc

    return error

def omega_dyn_const_error(constants, this: gtsam.CustomFactor,
               values: gtsam.Values,
               jacobians: Optional[List[np.ndarray]]) -> np.ndarray:

    last_state = constants[0]
    state = constants[1]

    # Extract constants from state
    last_theta = gtsam.Point3(0,0,2*np.arccos(last_state['rot'].w))
    theta = gtsam.Point3(0,0,2*np.arccos(state['rot'].w))
    dt = (state['stamp'].seconds + state['stamp'].nanos/10**9) - (last_state['stamp'].seconds + last_state['stamp'].nanos/10**9)
    
    # Get variable values
    last_omega_key = this.keys()[0]
    omega_key = this.keys()[1]
    last_omega, omega = values.atVector(last_omega_key), values.atVector(omega_key)
        
    # Compute error
    error = (theta - (last_theta + dt*(last_omega + omega)/2))
    
    if jacobians is not None:
        jacobians[0] = -np.eye(3) # last acc
        jacobians[1] = -np.eye(3) # acc

    return error



def main():

    # Initialize graph, variables, and noise
    graph = gtsam.NonlinearFactorGraph()
    init_values = gtsam.Values()
    unit_noise = gtsam.noiseModel.Diagonal.Sigmas(gtsam.Point3(1,1,1))

    # Initialize graph variable indices
    var_idx = 0

    # TODO - populate graph, see 

    # TODO - solve graph



if __name__ == "__main__":
    main()