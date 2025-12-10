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

    rviz_path = os.path.join(get_package_share_directory('multi_robot_simulator'), 'rviz', 'multi_robot_simulator_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', rviz_path
        ]
    )

    ld.add_action(rviz_node)
        
    return ld

