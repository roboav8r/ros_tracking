import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
# TODO - programmatically find bag path

def generate_launch_description():
    ld = LaunchDescription()

    # Get configuration from yaml file
    config = os.path.join(
        get_package_share_directory('ros_tracking'),
        'config',
        'nuscenes.yaml'
        )
    
    # Play bag
    bag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            "/home/jd/nuscenes2mcap/mcap/NuScenes-v1.0-mini-scene-0061-megvii/NuScenes-v1.0-mini-scene-0061-megvii_0.mcap",
        ],
        output="screen",
    )
    ld.add_action(bag)

    # Detection conversion node
    det_node = Node(
        package='ros_tracking',
        executable='det_converter',
        name='nuscenes_converter',
        remappings=[('/nuscenes_detections','/detections' )]
    )
    ld.add_action(det_node)

    # Manager node
    comp_node = Node(
        package='ros_tracking',
        executable='compute_nuscenes_results.py',
        name='compute_nuscenes_node',
        parameters=[config]
    )
    ld.add_action(comp_node)

    # Tracker node
    # trk_node = Node(
    #     package='ros_tracking',
    #     executable='py_tracker.py',
    #     name='tracker',
    #     output='screen',
    #     remappings=[('/detections','/converted_detections')],
    #     parameters=[config]
    # )
    # ld.add_action(trk_node)

    # Foxglove bridge for visualization
    bridge = IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('foxglove_bridge'),
                    'launch/foxglove_bridge_launch.xml'))
    )
    ld.add_action(bridge)

    return ld