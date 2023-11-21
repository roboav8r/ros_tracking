# TO DO
# - move factors to separate file and consolidate
# - move parameters to yaml, allow command line inputs
# - add multiple files to same graph
# - investigate blacklist items

import os
import json, yaml
import gtsam
import numpy as np
import pandas as pd
from functools import partial
from typing import List, Optional

# Constants
input_path = '/home/jd/tracking_ws/src/ros_tracking/data/annotations'
output_path = '/home/jd/tracking_ws/src/ros_tracking/data/variables'
blacklist = ['ac26','085f', 'cfd5', '8524']
motion_model_path = '/home/jd/tracking_ws/src/ros_tracking/config/motion_models.yaml'
motion_models = yaml.safe_load(open(motion_model_path))

# Custom factors
def const_pos_error(constants, this: gtsam.CustomFactor,
               values: gtsam.Values,
               jacobians: Optional[List[np.ndarray]]) -> np.ndarray:

    # Get position annotation
    state = constants[0]
    p = gtsam.Point3(state['pos']['x'],state['pos']['y'],state['pos']['z'])
    
    # Get variable values
    pos_key = this.keys()[0]
    pos = values.atVector(pos_key)
        
    # Compute error
    error = pos - p
    
    if jacobians is not None:
        jacobians[0] = np.eye(3)

    return error

def const_size_error(constants, this: gtsam.CustomFactor,
               values: gtsam.Values,
               jacobians: Optional[List[np.ndarray]]) -> np.ndarray:

    # Get position annotation
    state = constants[0]
    s = gtsam.Point3(state['size']['x'],state['size']['y'],state['size']['z'])
    
    # Get variable values
    size_key = this.keys()[0]
    size = values.atVector(size_key)
        
    # Compute error
    error = size - s
    
    if jacobians is not None:
        jacobians[0] = np.eye(3)

    return error

def vel_dyn_const_error(constants, this: gtsam.CustomFactor,
               values: gtsam.Values,
               jacobians: Optional[List[np.ndarray]]) -> np.ndarray:

    last_state = constants[0]
    state = constants[1]

    # Extract constants from state
    last_p = gtsam.Point3(last_state['pos']['x'],last_state['pos']['y'],last_state['pos']['z'])
    p = gtsam.Point3(state['pos']['x'],state['pos']['y'],state['pos']['z'])
    dt = (state['stamp']['seconds'] + state['stamp']['nanos']/10**9) - (last_state['stamp']['seconds'] + last_state['stamp']['nanos']/10**9)
    
    R = gtsam.Rot3(state['rot']['w'],0,0,state['rot']['z'])
    last_R = gtsam.Rot3(last_state['rot']['w'],0,0,last_state['rot']['z'])    
    
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
    dt = (state['stamp']['seconds'] + state['stamp']['nanos']/10**9) - (last_state['stamp']['seconds'] + last_state['stamp']['nanos']/10**9)
    
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
    last_theta = gtsam.Point3(0,0,2*np.arccos(last_state['rot']['w']))
    theta = gtsam.Point3(0,0,2*np.arccos(state['rot']['w']))
    dt = (state['stamp']['seconds'] + state['stamp']['nanos']/10**9) - (last_state['stamp']['seconds'] + last_state['stamp']['nanos']/10**9)
    
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

def xtrack_error(this: gtsam.CustomFactor,
               values: gtsam.Values,
               jacobians: Optional[List[np.ndarray]]) -> np.ndarray:

    # Get variable values
    vel_key = this.keys()[0]
    vel = values.atVector(vel_key)
        
    # Compute error
    error = np.array([0, vel[1], 0])
    
    if jacobians is not None:
        jacobians[0] = np.eye(3)

    return error

# MAIN LOOP
def main():

    # For each scene_objects dictionary
    for root, _, files in os.walk(input_path,topdown=True):

        for file in files:
            dataset = os.path.split(root)[-1]
            print('Handling dataset: ')
            print(dataset)
            print(root)

            # Initialize variables for this dataset
            variable_df = pd.DataFrame(columns=['scene','obj','class','attribute','vel_x','vel_y','vel_z','omega_z','beta','acc_x','acc_y','acc_z','dp_x','dp_y','dp_z','ds_x','ds_y','ds_z'])

            for file in files:

                # Get input filepath
                filepath = os.path.join(root,file)
                objects = json.load(open(filepath))
                scene = os.path.splitext(file)[0]

                print('Handling scene:')
                print(scene)
                print()

                # INITIALIZE: graph, variables, and noise for this scene
                graph = gtsam.NonlinearFactorGraph()
                init_values = gtsam.Values()
                unit_noise = gtsam.noiseModel.Diagonal.Sigmas(gtsam.Point3(1,1,1))
                var_idx = 0

                # POPULATE: Graph
                print('Populating graph')
                for key in objects: # For each object in scene

                    # Some keys just fail, figure out why later, bypass for now
                    if key in blacklist:
                        continue
                    
                    # Handle object in the scene
                    obj = objects[key]
                    model_type = motion_models[obj['category']]

                    if model_type=='static':

                        init_values.insert(gtsam.symbol('p',var_idx),gtsam.Point3(0,0,0))
                        init_values.insert(gtsam.symbol('s',var_idx),gtsam.Point3(0,0,0))
                        p_sym = gtsam.symbol('p',var_idx)
                        s_sym = gtsam.symbol('s',var_idx)

                        # iterate through states in object, add to graph 
                        for epoch in list(obj['states']):
                            obj['states'][epoch]['p_sym'] = p_sym
                            obj['states'][epoch]['s_sym'] = s_sym
                            const_pos_factor= gtsam.CustomFactor(unit_noise,[p_sym],partial(const_pos_error, [obj['states'][epoch]]))
                            const_size_factor= gtsam.CustomFactor(unit_noise,[s_sym],partial(const_size_error, [obj['states'][epoch]]))
                            graph.add(const_pos_factor)
                            graph.add(const_size_factor)
                            
                        var_idx+=1    

                    elif model_type in ['dynamic','ackermann']:

                        init_values.insert(gtsam.symbol('s',var_idx),gtsam.Point3(0,0,0))
                        s_sym = gtsam.symbol('s',var_idx)

                        for epoch in list(obj['states']):

                            obj['states'][epoch]['s_sym'] = s_sym
                            const_size_factor= gtsam.CustomFactor(unit_noise,[s_sym],partial(const_size_error, [obj['states'][epoch]]))
                            graph.add(const_size_factor)

                            # First trajectory element
                            if epoch == list(obj['states'])[0]:

                                # Add variables 
                                init_values.insert(gtsam.symbol('v',var_idx),gtsam.Point3(0,0,0))
                                init_values.insert(gtsam.symbol('w',var_idx),gtsam.Point3(0,0,0))
                                init_values.insert(gtsam.symbol('a',var_idx),gtsam.Point3(0,0,0))
                                obj['states'][epoch]['v_sym'] = gtsam.symbol('v',var_idx)
                                obj['states'][epoch]['w_sym'] = gtsam.symbol('w',var_idx)
                                obj['states'][epoch]['a_sym'] = gtsam.symbol('a',var_idx)
                                if model_type=='ackermann':      
                                    init_values.insert(gtsam.symbol('b',var_idx),np.array([[0]]))           
                                    obj['states'][epoch]['b_sym'] = gtsam.symbol('b',var_idx)

                                # Increment keys and indices
                                last_epoch = epoch
                                last_var_idx = var_idx
                                var_idx+=1

                            # Last trajectory element
                            elif epoch == list(obj['states'])[-1]:

                                # Add variables from this state
                                init_values.insert(gtsam.symbol('v',var_idx),gtsam.Point3(0,0,0))
                                init_values.insert(gtsam.symbol('w',var_idx),gtsam.Point3(0,0,0))
                                init_values.insert(gtsam.symbol('a',var_idx),gtsam.Point3(0,0,0))
                                obj['states'][epoch]['v_sym'] = gtsam.symbol('v',var_idx)
                                obj['states'][epoch]['w_sym'] = gtsam.symbol('w',var_idx)
                                obj['states'][epoch]['a_sym'] = gtsam.symbol('a',var_idx)
                                if model_type=='ackermann':          
                                    init_values.insert(gtsam.symbol('b',var_idx),np.array([[0]]))        
                                    obj['states'][epoch]['b_sym'] = gtsam.symbol('b',var_idx)

                                # Add Dynamics constraints
                                vel_dyn_const = gtsam.CustomFactor(unit_noise,[gtsam.symbol('v',last_var_idx),gtsam.symbol('v',var_idx)],partial(vel_dyn_const_error, [obj['states'][last_epoch],obj['states'][epoch]]))
                                graph.add(vel_dyn_const)
                                acc_dyn_const= gtsam.CustomFactor(unit_noise,[gtsam.symbol('v',last_var_idx),gtsam.symbol('v',var_idx),gtsam.symbol('a',last_var_idx),gtsam.symbol('a',var_idx)],partial(acc_dyn_const_error, [obj['states'][last_epoch],obj['states'][epoch]]))
                                graph.add(acc_dyn_const)
                                omega_dyn_const= gtsam.CustomFactor(unit_noise,[gtsam.symbol('w',last_var_idx),gtsam.symbol('w',var_idx)],partial(omega_dyn_const_error, [obj['states'][last_epoch],obj['states'][epoch]]))
                                graph.add(omega_dyn_const)

                                # For ackermann model, zero the cross-track velocity component
                                if model_type=='ackermann':
                                    xtrack_const = gtsam.CustomFactor(unit_noise,[gtsam.symbol('v',var_idx)],partial(xtrack_error))
                                    graph.add(xtrack_const)

                                # Add equality constraints
                                acc_eq_const = gtsam.CustomFactor(unit_noise,[gtsam.symbol('a',last_var_idx),gtsam.symbol('a',var_idx)],partial(acc_eq_error))
                                graph.add(acc_eq_const)
                                omega_eq_const = gtsam.CustomFactor(unit_noise,[gtsam.symbol('w',last_var_idx),gtsam.symbol('w',var_idx)],partial(vel_eq_error))
                                graph.add(omega_eq_const)
                                if len(obj['states'])==2: # If there are only 2 points in trajectory, add constant velocity constraint to avoid underdetermination
                                    vel_eq_const = gtsam.CustomFactor(unit_noise,[gtsam.symbol('v',last_var_idx),gtsam.symbol('v',var_idx)],partial(vel_eq_error))
                                    graph.add(vel_eq_const)

                                # Increment counters
                                last_epoch = epoch
                                last_var_idx = var_idx
                                var_idx+=1    

                            # Subsequent trajectory elements
                            else:                 

                                # Add variables from this state
                                init_values.insert(gtsam.symbol('v',var_idx),gtsam.Point3(0,0,0))
                                init_values.insert(gtsam.symbol('w',var_idx),gtsam.Point3(0,0,0))
                                init_values.insert(gtsam.symbol('a',var_idx),gtsam.Point3(0,0,0))
                                obj['states'][epoch]['v_sym'] = gtsam.symbol('v',var_idx)
                                obj['states'][epoch]['w_sym'] = gtsam.symbol('w',var_idx)
                                obj['states'][epoch]['a_sym'] = gtsam.symbol('a',var_idx)
                                if model_type=='ackermann':         
                                    init_values.insert(gtsam.symbol('b',var_idx),np.array([[0]]))           
                                    obj['states'][epoch]['b_sym'] = gtsam.symbol('b',var_idx)

                                # Add factor between last state and this state
                                vel_dyn_const= gtsam.CustomFactor(unit_noise,[gtsam.symbol('v',last_var_idx),gtsam.symbol('v',var_idx)],partial(vel_dyn_const_error, [obj['states'][last_epoch],obj['states'][epoch]]))
                                graph.add(vel_dyn_const)
                                acc_dyn_const= gtsam.CustomFactor(unit_noise,[gtsam.symbol('v',last_var_idx),gtsam.symbol('v',var_idx),gtsam.symbol('a',last_var_idx),gtsam.symbol('a',var_idx)],partial(acc_dyn_const_error, [obj['states'][last_epoch],obj['states'][epoch]]))
                                graph.add(acc_dyn_const)
                                omega_dyn_const= gtsam.CustomFactor(unit_noise,[gtsam.symbol('w',last_var_idx),gtsam.symbol('w',var_idx)],partial(omega_dyn_const_error, [obj['states'][last_epoch],obj['states'][epoch]]))
                                graph.add(omega_dyn_const)

                                # For ackermann model, zero the cross-track velocity component
                                if model_type=='ackermann':
                                    xtrack_const = gtsam.CustomFactor(unit_noise,[gtsam.symbol('v',var_idx)],partial(xtrack_error))
                                    graph.add(xtrack_const)

                                # Increment counters
                                last_epoch = epoch
                                last_var_idx = var_idx
                                var_idx+=1

                    else:
                        # TODO - make this an exception
                        print('No motion model')
            
                # SOLVE GRAPH for this scene
                print("Solving graph")
                params = gtsam.GaussNewtonParams()
                optimizer = gtsam.GaussNewtonOptimizer(graph, init_values, params)
                result = optimizer.optimize()

                # for scene/bag in dataset
                print("Adding variables to dataframe")
                for key in objects:
                    obj = objects[key]
                    model_type = motion_models[obj['category']]

                    if key in blacklist:
                        continue
                    
                    # Static model
                    if model_type == 'static':

                        # for state in object
                        for epoch in obj['states']:

                            # get solved data points for object + attribute, add to dataframe
                            data = pd.DataFrame([[scene,
                                                  key,
                                                  obj['category'], 
                                                  obj['states'][epoch]['att'] if obj['states'][epoch]['att'] else 'default',
                                                  0,0,0,
                                                  0,
                                                  0,
                                                  0,0,0,
                                                  result.atVector(obj['states'][epoch]['p_sym'])[0]-obj['states'][epoch]['pos']['x'],
                                                  result.atVector(obj['states'][epoch]['p_sym'])[1]-obj['states'][epoch]['pos']['y'],
                                                  result.atVector(obj['states'][epoch]['p_sym'])[2]-obj['states'][epoch]['pos']['z'],
                                                  result.atVector(obj['states'][epoch]['s_sym'])[0]-obj['states'][epoch]['size']['x'],
                                                  result.atVector(obj['states'][epoch]['s_sym'])[1]-obj['states'][epoch]['size']['y'],
                                                  result.atVector(obj['states'][epoch]['s_sym'])[2]-obj['states'][epoch]['size']['z']],
                                                  ], 
                                                  columns=['scene','obj','class','attribute','vel_x','vel_y','vel_z','omega_z','beta','acc_x','acc_y','acc_z','dp_x','dp_y','dp_z','ds_x','ds_y','ds_z'])
                            variable_df = pd.concat([variable_df,data],ignore_index=True)
                            
                    # Dynamic model
                    elif model_type in ['dynamic', 'ackermann']:
                    
                        # for state in object
                        for epoch in obj['states']:

                            # get solved data points for object + attribute, add to dataframe
                            data = pd.DataFrame([[scene,
                                                  key,
                                                  obj['category'], 
                                                  obj['states'][epoch]['att'] if obj['states'][epoch]['att'] else 'default',
                                                  result.atVector(obj['states'][epoch]['v_sym'])[0],
                                                  result.atVector(obj['states'][epoch]['v_sym'])[1],
                                                  result.atVector(obj['states'][epoch]['v_sym'])[2],
                                                  result.atVector(obj['states'][epoch]['w_sym'])[2],
                                                  np.arctan(result.atVector(obj['states'][epoch]['w_sym'])[2]*obj['states'][epoch]['size']['x']/result.atVector(obj['states'][epoch]['v_sym'])[0]) if (model_type=='ackermann' and result.atVector(obj['states'][epoch]['v_sym'])[0]!=0) else 0,
                                                  result.atVector(obj['states'][epoch]['a_sym'])[0],
                                                  result.atVector(obj['states'][epoch]['a_sym'])[1],
                                                  result.atVector(obj['states'][epoch]['a_sym'])[2],
                                                  0,0,0,
                                                  result.atVector(obj['states'][epoch]['s_sym'])[0]-obj['states'][epoch]['size']['x'],
                                                  result.atVector(obj['states'][epoch]['s_sym'])[1]-obj['states'][epoch]['size']['y'],
                                                  result.atVector(obj['states'][epoch]['s_sym'])[2]-obj['states'][epoch]['size']['z']]], 
                                                  columns=['scene','obj','class','attribute','vel_x','vel_y','vel_z','omega_z','beta','acc_x','acc_y','acc_z','dp_x','dp_y','dp_z','ds_x','ds_y','ds_z'])
                            variable_df = pd.concat([variable_df,data],ignore_index=True)

                    else:
                        # TODO - make this an error/exception
                        print('No analysis for model type: ' + model_type)

            # GENERATE AND SAVE RESULTS FOR THIS DATASET
            print("Saving results")
            variable_df.to_csv(os.path.join(output_path,dataset + '.csv'),columns = ['scene','obj','class','attribute','vel_x','vel_y','vel_z','omega_z','beta','acc_x','acc_y','acc_z','dp_x','dp_y','dp_z','ds_x','ds_y','ds_z'])

if __name__ == "__main__":
    main()