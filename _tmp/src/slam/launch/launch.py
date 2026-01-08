import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    slam_dir = get_package_share_directory('slam')

    # Launch arguments

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )

    ld.add_action(log_level_launch_arg)

    # Configuration file
    config = os.path.join(
        slam_dir,
        "config",
        "params.yaml"
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Nodes
    slam_node = Node(
        package="slam",
        executable="slam",
        output='screen',
        parameters=[config],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ],
        remappings=remappings
    )

    ld.add_action(slam_node) 
    
    return ld

