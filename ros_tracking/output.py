# import tf2_ros
# import geometry_msgs
import numpy as np

from tracking_msgs.msg import Track3D, Tracks3D
from foxglove_msgs.msg import SceneEntity, SceneUpdate, ArrowPrimitive, CubePrimitive, TextPrimitive, KeyValuePair

def PublishTracks(tracker, pub_name):
    tracker.trks_msg = Tracks3D()
    tracker.trks_msg.header.frame_id = tracker.frame_id
    tracker.trks_msg.header.stamp = tracker.dets_msg.header.stamp

    for trk in tracker.trks:
        # Create track message
        trk_msg = Track3D()

        # Add track information to message
        trk_msg.time_created = trk.time_created.to_msg()
        trk_msg.time_updated = trk.time_updated.to_msg()
        trk_msg.track_id = trk.trk_id

        # Add spatial information to message
        trk_msg.pose.pose.position.x = trk.spatial_state.mean()[0]
        trk_msg.pose.pose.position.y = trk.spatial_state.mean()[1]
        trk_msg.pose.pose.position.z = trk.spatial_state.mean()[2]
        trk_msg.twist.twist.linear.x = trk.spatial_state.mean()[3]
        trk_msg.twist.twist.linear.y = trk.spatial_state.mean()[4]
        trk_msg.twist.twist.linear.z = trk.spatial_state.mean()[5]
       
        # TODO - add covariances and object size/bbox
        # trk_msg.pose.covariance =        
        # trk_msg.twist.covariance =
        # trk_msg.height = 
        # trk_msg.width = 
        # trk_msg.depth = 
    
        # Add semantic information to message
        trk_msg.class_confidence = trk.class_dist(trk.class_dist.argmax())
        trk_msg.class_string = tracker.object_classes[trk.class_dist.argmax()]

        tracker.trks_msg.tracks.append(trk_msg)

    # Publish populated message
    exec('tracker.%s.publish(tracker.trks_msg)' % pub_name)

def PublishScene(tracker, pub_name):
    # Create scene message
    tracker.scene_msg = SceneUpdate()

    for trk in tracker.trks:
        
        # if trk.track_conf(1) < tracker.pub_thresh: # not confident enough to publish
        #     continue

        # if tracker.object_classes[trk.class_dist.argmax()] == 'void_ignore':
        #     continue

        # Create track message
        entity_msg = SceneEntity()

        # Add track information to message
        entity_msg.frame_id = tracker.frame_id
        entity_msg.timestamp = tracker.dets_msg.header.stamp
        entity_msg.id = str(trk.trk_id)
        entity_msg.frame_locked = True
        entity_msg.lifetime.nanosec = 500000000

        # TODO - add covariances and object size/bbox
        cube = CubePrimitive()
        cube.pose.position.x = trk.spatial_state.mean()[0]
        cube.pose.position.y = trk.spatial_state.mean()[1]
        cube.pose.position.z = trk.spatial_state.mean()[2]

        # TODO make helper function for rpy->quat
        cr = np.cos(0.5*trk.spatial_state.mean()[6])
        sr = np.sin(0.5*trk.spatial_state.mean()[6])
        cp = np.cos(0.5*trk.spatial_state.mean()[7])
        sp = np.sin(0.5*trk.spatial_state.mean()[7])
        cy = np.cos(0.5*trk.spatial_state.mean()[8])
        sy = np.sin(0.5*trk.spatial_state.mean()[8])

        # trk.orientation = gtsam.Rot3(cr*cp*cy + sr*sp*sy,
        #                               sr*cp*cy - cr*sp*sy,
        #                               cr*sp*cy + sr*cp*sy,
        #                               cr*cp*sy - sr*sp*cy)
        cube.pose.orientation.w = cr*cp*cy + sr*sp*sy
        cube.pose.orientation.x = sr*cp*cy - cr*sp*sy
        cube.pose.orientation.y = cr*sp*cy + sr*cp*sy
        cube.pose.orientation.z = cr*cp*sy - sr*sp*cy
        cube.size.x = trk.spatial_state.mean()[12]
        cube.size.y = trk.spatial_state.mean()[13]
        cube.size.z = trk.spatial_state.mean()[14]
        cube.color.a = 0.1
        entity_msg.cubes.append(cube)

        # Add velocity information to message
        # vel_quat = Quaternion(trk.spatial_state.mean()[3],trk.spatial_state.mean()[4],trk.spatial_state.mean()[5],0)
        # vel_mag = vel_quat.length()
        # vel_quat.normalize()
        vel_arrow_x = ArrowPrimitive()
        vel_arrow_x.pose = cube.pose
        vel_arrow_x.shaft_length = trk.spatial_state.mean()[3]
        vel_arrow_x.shaft_diameter = .1
        vel_arrow_x.head_length = .2
        vel_arrow_x.shaft_diameter = .1
        vel_arrow_x.color.a = 0.5
        entity_msg.arrows.append(vel_arrow_x)


        # TODO - Add semantic information to message
        text = TextPrimitive()
        text.billboard = True
        text.font_size = 12.
        text.scale_invariant = True
        text.color.a = 1.0
        text.pose.position.x = trk.spatial_state.mean()[0]
        text.pose.position.y = trk.spatial_state.mean()[1]
        text.pose.position.z = trk.spatial_state.mean()[2]
        text.text = "%s-%.0f: %.0f %%" % (tracker.object_classes[trk.class_dist.argmax()], trk.trk_id, trk.class_dist(trk.class_dist.argmax())*100)
        entity_msg.texts.append(text)

        # Add metadata
        name_md = KeyValuePair()
        name_md.key = 'class_name'
        name_md.value = tracker.object_classes[trk.class_dist.argmax()]
        entity_msg.metadata.append(name_md)

        score_md = KeyValuePair()
        score_md.key = 'class_score'
        score_md.value = str(trk.class_dist(trk.class_dist.argmax())*trk.class_conf) # TODO - this is temporary... right?
        entity_msg.metadata.append(score_md)

        att_md = KeyValuePair()
        att_md.key = 'attribute'
        att_md.value = '' # TODO add attribute
        entity_msg.metadata.append(att_md)

        sample_md = KeyValuePair()
        sample_md.key = trk.metadata[0].key
        sample_md.value = trk.metadata[0].value
        entity_msg.metadata.append(sample_md)

        trk_md = KeyValuePair()
        trk_md.key = 'track_score'
        trk_md.value = str(trk.track_conf(1))
        entity_msg.metadata.append(trk_md)

        tracker.scene_msg.entities.append(entity_msg)
    
    # Publish scene message
    exec('tracker.%s.publish(tracker.scene_msg)' % pub_name)