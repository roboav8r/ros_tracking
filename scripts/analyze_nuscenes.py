import pandas as pd
import numpy as np
import os
import json
import yaml

# Parameters
solver_data_path = '/home/jd/tracking_ws/src/ros_tracking/data/solver'
motion_model_path = '/home/jd/tracking_ws/src/ros_tracking/config/motion_models.yaml'
motion_models = yaml.safe_load(open(motion_model_path))
motion_param_path = '/home/jd/tracking_ws/src/ros_tracking/config/motion_params.yaml'

# Load csv file into dataframe
solver_data_df = pd.DataFrame()

# Iterate through solver files and append to dataframe
print('Loading solved variable data')
for root, _,files in os.walk(solver_data_path,topdown=True):
    for file in files:

        # Get input filepath
        in_file = os.path.join(root,file)
        print(in_file)

        data = pd.read_csv(in_file)

        solver_data_df = pd.concat((solver_data_df,data))

print(solver_data_df)
# Find unique classes and attributes in the dataframe
class_att_dict = dict()
for class_type in solver_data_df['class'].unique():   
    class_att_dict[class_type] = list(solver_data_df.loc[solver_data_df['class']==class_type,'attribute'].unique())
print(class_att_dict)    


# Compute motion model parameters for each unique class, attribute
motion_params = dict()
for cls in class_att_dict:
    print(cls)

    if cls not in motion_params.keys():
        motion_params[cls] = dict()

    mm_type = motion_models[cls]
    print(mm_type)

    for att in class_att_dict[cls]:
        print(att)

        # Get motion model type
        if att not in motion_params[cls].keys():
            motion_params[cls][att] = dict()


        # Compute position variance for static (const position) model
        if mm_type=='static':
            px_var = np.var(solver_data_df.loc[(solver_data_df['class']==cls) & (solver_data_df['attribute']==att),'dp_x'])
            py_var = np.var(solver_data_df.loc[(solver_data_df['class']==cls) & (solver_data_df['attribute']==att),'dp_y'])
            pz_var = np.var(solver_data_df.loc[(solver_data_df['class']==cls) & (solver_data_df['attribute']==att),'dp_z'])

            motion_params[cls]['type'] = 'static'
            motion_params[cls][att]['px_var'] = float(px_var)
            motion_params[cls][att]['py_var'] = float(py_var)
            motion_params[cls][att]['pz_var'] = float(pz_var)

        elif mm_type in ['dynamic','ackermann']:

            vx_mean = np.mean(solver_data_df.loc[(solver_data_df['class']==cls) & (solver_data_df['attribute']==att),'vel_x'])
            vy_mean = np.mean(solver_data_df.loc[(solver_data_df['class']==cls) & (solver_data_df['attribute']==att),'vel_y'])
            vz_mean = np.mean(solver_data_df.loc[(solver_data_df['class']==cls) & (solver_data_df['attribute']==att),'vel_z'])
            omegaz_mean = np.mean(solver_data_df.loc[(solver_data_df['class']==cls) & (solver_data_df['attribute']==att),'omega_z'])
            
            vx_var = np.var(solver_data_df.loc[(solver_data_df['class']==cls) & (solver_data_df['attribute']==att),'vel_x'])
            vy_var = np.var(solver_data_df.loc[(solver_data_df['class']==cls) & (solver_data_df['attribute']==att),'vel_y'])
            vz_var = np.var(solver_data_df.loc[(solver_data_df['class']==cls) & (solver_data_df['attribute']==att),'vel_z'])
            omegaz_var = np.var(solver_data_df.loc[(solver_data_df['class']==cls) & (solver_data_df['attribute']==att),'omega_z'])

            motion_params[cls]['type'] = 'dynamic'
            motion_params[cls][att]['vx_mean'] = float(vx_mean)
            motion_params[cls][att]['vy_mean'] = float(vy_mean)
            motion_params[cls][att]['vz_mean'] = float(vz_mean)
            motion_params[cls][att]['vx_var'] = float(vx_var)
            motion_params[cls][att]['vy_var'] = float(vy_var)
            motion_params[cls][att]['vz_var'] = float(vz_var)
            motion_params[cls][att]['omegaz_mean'] = float(omegaz_mean)
            motion_params[cls][att]['omegaz_var'] = float(omegaz_var)


    print()

with open(motion_param_path, 'w') as outfile:
    yaml.dump(motion_params,outfile,indent=4)