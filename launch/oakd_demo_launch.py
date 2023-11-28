import os

from ament_index_python import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # Sensor node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('depthai_ros_driver'),
                    'launch',
                    'camera.launch.py'
                ])
            ]),
            launch_arguments={
                'params_file': PathJoinSubstitution([
                    FindPackageShare('ros_tracking'),
                    'config',
                    'oakd.yaml'
                ])
            }.items()
        ),

        # Detection conversion node
        Node(
            package='ros_tracking',
            executable='depthai_converter',
            name='depthai_converter',
            remappings=[('/depthai_detections','/oak/nn/spatial_detections')]
        ),

        # Foxglove bridge for visualization
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('foxglove_bridge'),
                    'launch/foxglove_bridge_launch.xml'))
        )

    ])
