import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from tracetools_launch.action import Trace

def generate_launch_description():
    ld = LaunchDescription()

    # Get configuration from yaml file
    exp_config = os.path.join(
        get_package_share_directory('ros_tracking'),
        'config',
        'experiments.yaml'
        )
    
    # trace = Trace(
    #     session_name="nuscenes_tracing",
    # )
    # ld.add_action(trace)

    # Manager node
    comp_node = Node(
        package='ros_tracking',
        executable='run_nuscenes_experiments.py',
        name='nusc_exp_mgr_node',
        output="screen",
        parameters=[exp_config]
    )
    ld.add_action(comp_node)

    return ld