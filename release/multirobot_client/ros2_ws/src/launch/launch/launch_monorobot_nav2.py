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

    # Launch descriptions

    turtlebot3_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot3_wrapper'), 'launch', 'robot.launch.py')
        ]),
        launch_arguments={
            'cam_pos_z': "0.235"
        }.items()
    )

    nav2_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_launch'), 'launch', 'launch.py')
        ]),
        launch_arguments={
            'slam_toolbox': 'False',
            'rviz': 'False',
        }.items()
    )

    ld.add_action(turtlebot3_launch_description)
    ld.add_action(nav2_launch_description)
        
    return ld

