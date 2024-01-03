
import os
import json

from rclpy.serialization import serialize_message
import rosbag2_py
from tracking_msgs.msg import Detection3D, Detections3D
from foxglove_msgs.msg import SceneUpdate, SceneEntity, CubePrimitive, TextPrimitive, KeyValuePair
from builtin_interfaces.msg import Time

from nuscenes.nuscenes import NuScenes
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

# Params
mcap_path = '/home/jd/nuscenes2mcap/output'
topics=['/markers/annotations','/markers/detections']
data_dir = '/home/jd/nuScenes/data'
state_output_dir = '/home/jd/tracking_ws/src/ros_tracking/data/annotations'
sensor_output_dir = '/home/jd/tracking_ws/src/ros_tracking/data/detections'
name="v1.0-mini"
frame_id = 'map'

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
                
                # ADD TO DETECTION DICTIONARY - indexed by time
                if current_epoch not in scene_detections.keys(): # Initialize entry if needed
                    scene_detections[current_epoch] = dict()
                    scene_detections[current_epoch]['stamp'] = t[current_epoch]
                    scene_detections[current_epoch]['objects'] = dict()
                    scene_detections[current_epoch]['detections'] = dict()

                if obj_id not in scene_detections[current_epoch]['objects'].keys():
                    scene_detections[current_epoch]['objects'][obj_id] = dict()

                # add object to detection dictionary
                scene_detections[current_epoch]['objects'][obj_id]['category'] = cat
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

                # ADD TO DETECTION DICTIONARY - indexed by time
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

def write_detections_to_mcap(det_dict, mcap_dir, filename):


    # Create writer
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=mcap_dir + '/mcap/'+ filename, storage_id='mcap'),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    # Create topics
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name="/detections3d", type="tracking_msgs/msg/Detections3D", serialization_format="cdr"
        )
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name="/detections3d_foxglove", type="foxglove_msgs/msg/SceneUpdate", serialization_format="cdr"
        )
    )


    # Add messages to topic
    for epoch_key in det_dict.keys():

        # Create empty Detections3D message
        dets_msg = Detections3D()
        scene_msg = SceneUpdate()

        timestamp = Time()
        timestamp.sec = det_dict[epoch_key]['stamp']['seconds']
        timestamp.nanosec = det_dict[epoch_key]['stamp']['nanos']

        dets_msg.header.stamp.sec = det_dict[epoch_key]['stamp']['seconds']
        dets_msg.header.stamp.nanosec = det_dict[epoch_key]['stamp']['nanos']
        dets_msg.header.frame_id = frame_id

        for det_key in det_dict[epoch_key]['detections']:

            # Populate detection message
            # print(det_dict[epoch_key]['detections'][det_key].keys()) # ['stamp', 'category', 'score', 'pos', 'rot', 'size', 'att']
            det_msg = Detection3D()

            det_msg.pose.position.x = det_dict[epoch_key]['detections'][det_key]['pos']['x']
            det_msg.pose.position.y = det_dict[epoch_key]['detections'][det_key]['pos']['y']
            det_msg.pose.position.z = det_dict[epoch_key]['detections'][det_key]['pos']['z']

            det_msg.pose.orientation.x = det_dict[epoch_key]['detections'][det_key]['rot']['x']
            det_msg.pose.orientation.y = det_dict[epoch_key]['detections'][det_key]['rot']['y']
            det_msg.pose.orientation.z = det_dict[epoch_key]['detections'][det_key]['rot']['z']
            det_msg.pose.orientation.w = det_dict[epoch_key]['detections'][det_key]['rot']['w']

            det_msg.class_string = det_dict[epoch_key]['detections'][det_key]['category']
            det_msg.class_confidence = float(det_dict[epoch_key]['detections'][det_key]['score'])
            det_msg.attribute = det_dict[epoch_key]['detections'][det_key]['att']

            det_msg.bbox.center = det_msg.pose
            det_msg.bbox.size.x = det_dict[epoch_key]['detections'][det_key]['size']['x']
            det_msg.bbox.size.y = det_dict[epoch_key]['detections'][det_key]['size']['y']
            det_msg.bbox.size.z = det_dict[epoch_key]['detections'][det_key]['size']['z']
            
            dets_msg.detections.append(det_msg)

            # Populate SceneUpdate message
            entity_msg = SceneEntity()
            entity_msg.timestamp = dets_msg.header.stamp
            entity_msg.frame_id = frame_id
            entity_msg.id = str(det_key)
            entity_msg.frame_locked = True
            entity_msg.lifetime.nanosec = 500000000 # just under half a second so detections clear before 2Hz 

            cube = CubePrimitive()
            cube.pose.position.x = det_dict[epoch_key]['detections'][det_key]['pos']['x']
            cube.pose.position.y = det_dict[epoch_key]['detections'][det_key]['pos']['y']
            cube.pose.position.z = det_dict[epoch_key]['detections'][det_key]['pos']['z']
            cube.pose.orientation.w = det_dict[epoch_key]['detections'][det_key]['rot']['w']
            cube.pose.orientation.x = det_dict[epoch_key]['detections'][det_key]['rot']['x']
            cube.pose.orientation.y = det_dict[epoch_key]['detections'][det_key]['rot']['y']
            cube.pose.orientation.z = det_dict[epoch_key]['detections'][det_key]['rot']['z']
            cube.size.x = det_dict[epoch_key]['detections'][det_key]['size']['x']
            cube.size.y = det_dict[epoch_key]['detections'][det_key]['size']['y']
            cube.size.z = det_dict[epoch_key]['detections'][det_key]['size']['z']
            # cube.color.r = c[0]
            # cube.color.g = c[1]
            # cube.color.b = c[2]
            # cube.color.a = det["detection_score"]
            entity_msg.cubes.append(cube)


            # TODO - Attribute

            # TODO - Category

            scene_msg.entities.append(entity_msg)


        # Write detections message to topic
        writer.write('/detections3d', serialize_message(dets_msg), dets_msg.header.stamp.sec*10**9 + dets_msg.header.stamp.nanosec)
        writer.write('/detections3d_foxglove', serialize_message(scene_msg), dets_msg.header.stamp.sec*10**9 + dets_msg.header.stamp.nanosec)


    # Close writer
    del writer

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

            # Convert to mcap and save
            write_detections_to_mcap(detection_dict, sensor_output_dir, os.path.splitext(file)[0])

if __name__ == "__main__":
    main()