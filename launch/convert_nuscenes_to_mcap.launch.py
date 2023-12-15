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
        'nuscenes_to_mcap.yaml'
        )
    
    # MCAP converter node
    comp_node = Node(
        package='ros_tracking',
        executable='nuscenes_to_mcap.py',
        name='nuscenes_to_mcap',
        output="screen",
        parameters=[config]
    )
    ld.add_action(comp_node)

    return ld