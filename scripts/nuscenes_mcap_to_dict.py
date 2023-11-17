
from nuscenes.nuscenes import NuScenes

import os
import json
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory
# import argparse
# from rclpy.serialization import deserialize_message
# from rosidl_runtime_py.utilities import get_message
# from std_msgs.msg import String
# import rosbag2_py
# 
# 

# import numpy as np
# import matplotlib.pyplot as plt
# import gtsam
# from functools import partial
# from typing import List, Optional
# import pandas as pd


# Params
mcap_path = '/home/jd/nuscenes2mcap/output'
topics=['/markers/annotations','/markers/detections']
data_dir = '/home/jd/nuscenes/data'
output_data_dir = '/home/jd/tracking_ws/src/ros_tracking/data/dict'
name="v1.0-mini"

# Create nuscenes object
nusc = NuScenes(version=name, dataroot=str(data_dir), verbose=True)


def mcap_to_dict(in_file):
    reader = make_reader(open(in_file, 'br'), decoder_factories=[DecoderFactory()])

    # Initialize
    epoch = 0
    t = dict()
    scene_objects = dict()

    # Iterate through messages
    for schema, channel, message, msg in reader.iter_decoded_messages(topics=topics):
            
        if channel.topic =='/markers/annotations':
            
            # Get timestamp for this epoch
            tstamp = msg.entities[0].timestamp
            sec = tstamp.seconds
            ns = tstamp.nanos
            if {'seconds': sec, 'nanos': ns} not in t.values():
                t[epoch] = dict()
                t[epoch]['seconds'] = sec
                t[epoch]['nanos'] = ns
                epoch+=1

            # Get epoch from timestamp
            current_epoch = list(t.keys())[list(t.values()).index({'seconds': tstamp.seconds, 'nanos': tstamp.nanos})]       
            
            for entity in msg.entities:
                # Get entity data
                obj_id = entity.id
                
                # Get spatial data
                pos, rot, size  = entity.cubes[0].pose.position, entity.cubes[0].pose.orientation, entity.cubes[0].size
                
                # Get semantic data
                cat = entity.metadata[0].value
                attribute = nusc.get('attribute',entity.metadata[1].value)['name'] if entity.metadata[1].value else ''
                
                # Initialize dict entry for this object if not done already
                if obj_id not in scene_objects.keys():
                    scene_objects[obj_id] = dict()
                    scene_objects[obj_id]['category'] = cat
                    scene_objects[obj_id]['states'] = dict()
            
                scene_objects[obj_id]['states'][current_epoch] = {
                        "stamp": {'seconds': tstamp.seconds, 'nanos': tstamp.nanos},
                        "pos": {'x': pos.x,'y': pos.y,'z': pos.z},
                        "rot": {'z': rot.z,'w': rot.w},
                        "size": {'x': size.x,'y': size.y,'z': size.z},
                        "att": attribute}

    return scene_objects

def main():
    for root, _,files in os.walk(mcap_path,topdown=True):
        for file in files:

            # Get input filepath
            in_file = os.path.join(root,file)

            # Convert to dictionary
            dict = mcap_to_dict(in_file)

            # Save to json
            with open(os.path.join(output_data_dir,os.path.splitext(file)[0] + '.json'), "w") as outfile:
                json.dump(dict,outfile)

if __name__ == "__main__":
    main()