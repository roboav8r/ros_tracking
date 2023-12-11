# import tf2_ros
# import geometry_msgs

from tracking_msgs.msg import Track3D, Tracks3D
from foxglove_msgs.msg import SceneEntity, SceneUpdate, ArrowPrimitive, CubePrimitive, TextPrimitive

def PublishTracks(tracker):
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
    tracker.track_pub.publish(tracker.trks_msg)

def PublishScene(tracker):
    # Create scene message
    tracker.scene_msg = SceneUpdate()

    for trk in tracker.trks:
        # Create track message
        entity_msg = SceneEntity()

        # Add track information to message
        entity_msg.frame_id = tracker.frame_id
        entity_msg.timestamp = tracker.dets_msg.header.stamp
        entity_msg.id = str(trk.trk_id)
        entity_msg.frame_locked = True
        entity_msg.lifetime.nanosec = 500000000

        # Add spatial information to message
        # vel_quat = Quaternion(trk.spatial_state.mean()[3],trk.spatial_state.mean()[4],trk.spatial_state.mean()[5],0)
        # vel_mag = vel_quat.length()
        # vel_quat.normalize()
        # vel_arrow = ArrowPrimitive()
        # vel_arrow.pose.position = trk.spatial_state.mean()
        # vel_arrow.pose.orientation = geometry_msgs.msg.Quaternion(vel_quat)
        # vel_arrow.shaft_length = vel_mag
        # entity_msg.arrows.append(vel_arrow)

        # TODO - add covariances and object size/bbox
        cube = CubePrimitive()
        cube.pose.position.x = trk.spatial_state.mean()[0]
        cube.pose.position.y = trk.spatial_state.mean()[1]
        cube.pose.position.z = trk.spatial_state.mean()[2]
        cube.size.x = 0.25
        cube.size.y = 0.25
        cube.size.z = 0.25
        cube.color.a = 1.0
        entity_msg.cubes.append(cube)

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

        # Textprimitive
        # metadata
        tracker.scene_msg.entities.append(entity_msg)
    
    # Publish scene message
    tracker.scene_pub.publish(tracker.scene_msg)
    