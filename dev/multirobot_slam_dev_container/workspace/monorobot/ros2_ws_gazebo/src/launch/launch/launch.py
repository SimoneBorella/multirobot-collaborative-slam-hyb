import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    # Launch arguments

    robots_config_file_launch_arg = DeclareLaunchArgument(
        'robots_config_file',
        default_value=os.path.join(get_package_share_directory('launch'), 'config', 'robots.yaml')
    )

    rviz_gz_launch_arg = DeclareLaunchArgument(
        'rviz_gz',
        default_value='False'
    )

    rviz_nav2_launch_arg = DeclareLaunchArgument(
        'rviz_nav2',
        default_value='True'
    )

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )




    # Load robots config
    with open(os.path.join(get_package_share_directory('launch'), 'config', 'robots.yaml'), "r") as file:
        robots_config = yaml.safe_load(file)




    # Launch descriptions

    ros_gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_bringup'), 'launch','launch.py')
        ]),
        launch_arguments={
            'robots_config_file': LaunchConfiguration('robots_config_file'),
            'rviz': LaunchConfiguration('rviz_gz'),
            'log_level': LaunchConfiguration('log_level'),
        }.items()
    )

    nav2_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_launch'), 'launch', 'launch.py')
        ]),
        launch_arguments={
            'robots_config_file': LaunchConfiguration('robots_config_file'),
            'rviz': LaunchConfiguration('rviz_nav2'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    # Launch arguments
    ld.add_action(robots_config_file_launch_arg)
    ld.add_action(rviz_gz_launch_arg)
    ld.add_action(rviz_nav2_launch_arg)
    ld.add_action(log_level_launch_arg)

    # Launch descriptions
    ld.add_action(ros_gz_launch_description)
    ld.add_action(nav2_launch_description)
    
    return ld

