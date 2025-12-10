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

    # Launch arguments

    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=''
    )

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )

    ld.add_action(namespace_launch_arg)
    ld.add_action(log_level_launch_arg)


    # Launch descriptions

    turtlebot3_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot3_wrapper'), 'launch', 'robot.launch.py')
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )


    camera_calibration_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sensor_processing'), 'launch', 'camera_calibration.launch.py')
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )


    ld.add_action(turtlebot3_launch_description)
    ld.add_action(camera_calibration_launch_description)
    
    return ld

