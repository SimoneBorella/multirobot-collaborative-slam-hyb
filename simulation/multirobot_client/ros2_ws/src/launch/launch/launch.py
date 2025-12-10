import os
import yaml
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    # Launch arguments

    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=''
    )

    launch_nav2_launch_arg = DeclareLaunchArgument(
        'launch_nav2',
        default_value='False'
    )

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )



    ld.add_action(namespace_launch_arg)
    ld.add_action(launch_nav2_launch_arg)
    ld.add_action(log_level_launch_arg)


    # Launch descriptions

    turtlebot3_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot3_wrapper'), 'launch', 'robot.launch.py')
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'log_level': LaunchConfiguration('log_level'),
            'cam_pos_z': "0.235"
        }.items(),
    )

    sensor_processing_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sensor_processing'), 'launch', 'launch.py')
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    nav2_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_launch'), 'launch', 'launch.py')
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'slam_toolbox': 'False',
            'rviz': 'False',
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_nav2'))
    )

    ld.add_action(turtlebot3_launch_description)
    ld.add_action(sensor_processing_launch_description)
    ld.add_action(nav2_launch_description)
        
    return ld

