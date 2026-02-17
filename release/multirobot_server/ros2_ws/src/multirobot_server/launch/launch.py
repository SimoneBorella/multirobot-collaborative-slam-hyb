import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression, EqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterFile

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    multirobot_server_dir = get_package_share_directory('multirobot_server')

    # Launch arguments
    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )

    ld.add_action(log_level_launch_arg)

    # Configuration file

    config = os.path.join(
        multirobot_server_dir,
        "config",
        "params.yaml"
    )

    # Nodes
    multirobot_server_node = Node(
        package="multirobot_server",
        executable="multirobot_server",
        output='screen',
        parameters=[config],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ]
    )

    ld.add_action(multirobot_server_node)

    return ld

