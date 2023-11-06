import os

from ament_index_python import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # Sensor node
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('depthai_ros_driver'),
                'launch',
                'camera.launch.py'))
        ),

        # Foxglove bridge for visualization
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('foxglove_bridge'),
                    'launch/foxglove_bridge_launch.xml'))
        )

    ])