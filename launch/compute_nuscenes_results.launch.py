import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
# TODO - programmatically find bag path

def generate_launch_description():
    ld = LaunchDescription()

    bag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            "--loop",
            "/home/jd/tracking_ws/src/ros_tracking/data/detections/mcap/NuScenes-v1.0-mini-scene-0061-megvii/NuScenes-v1.0-mini-scene-0061-megvii_0.mcap",
        ],
        output="screen",
    )
    ld.add_action(bag)

    # Foxglove bridge for visualization
    bridge = IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('foxglove_bridge'),
                    'launch/foxglove_bridge_launch.xml'))
    )
    ld.add_action(bridge)

    return ld