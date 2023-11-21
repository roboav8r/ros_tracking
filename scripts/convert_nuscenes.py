
from nuscenes.nuscenes import NuScenes

import os
import json
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

# Params
mcap_path = '/home/jd/nuscenes2mcap/output'
topics=['/markers/annotations','/markers/detections']
data_dir = '/home/jd/nuScenes/data'
state_output_dir = '/home/jd/tracking_ws/src/ros_tracking/data/annotations'
sensor_output_dir = '/home/jd/tracking_ws/src/ros_tracking/data/detections'
name="v1.0-mini"

# Create nuscenes object
nusc = NuScenes(version=name, dataroot=str(data_dir), verbose=True)

def mcap_to_dict(in_file):
    reader = make_reader(open(in_file, 'br'), decoder_factories=[DecoderFactory()])

    # Initialize
    epoch = 0
    t = dict()
    scene_states = dict()
    scene_detections = dict()

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
            
            # Lookup current epoch from timestamp
            current_epoch = list(t.keys())[list(t.values()).index({'seconds': tstamp.seconds, 'nanos': tstamp.nanos})]       
            
            for entity in msg.entities:
                # Get entity data
                obj_id = entity.id
                
                # Get spatial data
                pos, rot, size  = entity.cubes[0].pose.position, entity.cubes[0].pose.orientation, entity.cubes[0].size
                
                # Get semantic data
                cat = entity.metadata[0].value
                attribute = nusc.get('attribute',entity.metadata[1].value)['name'] if entity.metadata[1].value else ''
                
                # ADD TO STATE TRANSITION DICTIONARY - indexed by object
                # Initialize dict entry for this object if not done already
                if obj_id not in scene_states.keys():
                    scene_states[obj_id] = dict()
                    scene_states[obj_id]['category'] = cat
                    scene_states[obj_id]['states'] = dict()
            
                scene_states[obj_id]['states'][current_epoch] = {
                        "stamp": {'seconds': tstamp.seconds, 'nanos': tstamp.nanos},
                        "pos": {'x': pos.x,'y': pos.y,'z': pos.z},
                        "rot": {'z': rot.z,'w': rot.w},
                        "size": {'x': size.x,'y': size.y,'z': size.z},
                        "att": attribute}
                
                # ADD TO DETECTION DICTIONARY - indexed by object
                if current_epoch not in scene_detections.keys(): # Initialize entry if needed
                    scene_detections[current_epoch] = dict()
                    scene_detections[current_epoch]['stamp'] = t[current_epoch]
                    scene_detections[current_epoch]['objects'] = dict()
                    scene_detections[current_epoch]['detections'] = dict()

                if obj_id not in scene_detections[current_epoch]['objects'].keys():
                    scene_detections[current_epoch]['objects'][obj_id] = dict()

                # add object to detection dictionary
                scene_detections[current_epoch]['objects'][obj_id]['state'] = {
                        "stamp": {'seconds': tstamp.seconds, 'nanos': tstamp.nanos},
                        "pos": {'x': pos.x,'y': pos.y,'z': pos.z},
                        "rot": {'z': rot.z,'w': rot.w},
                        "size": {'x': size.x,'y': size.y,'z': size.z},
                        "att": attribute}


        if channel.topic =='/markers/detections':
            
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
                det_id = entity.id
                
                # Get spatial data
                pos, rot, size  = entity.cubes[0].pose.position, entity.cubes[0].pose.orientation, entity.cubes[0].size
                
                # Get semantic data
                cat = entity.metadata[0].value
                score = entity.metadata[1].value
                attribute = entity.metadata[1].value if entity.metadata[1].value else '' 

                # ADD TO DETECTION DICTIONARY - indexed by object
                if current_epoch not in scene_detections.keys(): # Initialize entry if needed
                    scene_detections[current_epoch] = dict()
                    scene_detections[current_epoch]['stamp'] = t[current_epoch]
                    scene_detections[current_epoch]['objects'] = dict()
                    scene_detections[current_epoch]['detections'] = dict()

                if det_id not in scene_detections[current_epoch]['objects'].keys():
                    scene_detections[current_epoch]['detections'][det_id] = dict()

                # add object to detection dictionary
                scene_detections[current_epoch]['detections'][det_id] = {
                        "stamp": {'seconds': tstamp.seconds, 'nanos': tstamp.nanos},
                        "category": cat,
                        "score": score,
                        "pos": {'x': pos.x,'y': pos.y,'z': pos.z},
                        "rot": {'x': rot.x,'y': rot.y, 'z': rot.z, 'w': rot.w},
                        "size": {'x': size.x,'y': size.y,'z': size.z},
                        "att": attribute}

    return scene_states, scene_detections

def main():
    for root, _,files in os.walk(mcap_path,topdown=True):
        for file in files:

            # Get input filepath
            in_file = os.path.join(root,file)

            # Convert to dictionary
            state_dict, detection_dict = mcap_to_dict(in_file)

            # Save files to json
            with open(os.path.join(state_output_dir,os.path.splitext(file)[0] + '.json'), "w") as outfile:
                json.dump(state_dict,outfile)
            
            with open(os.path.join(sensor_output_dir,os.path.splitext(file)[0] + '.json'), "w") as outfile:
                json.dump(detection_dict,outfile)

if __name__ == "__main__":
    main()