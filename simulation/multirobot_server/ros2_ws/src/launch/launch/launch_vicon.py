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

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False'
    )

    verbose_launch_arg = DeclareLaunchArgument(
        'verbose',
        default_value='False'
    )

    ld.add_action(rviz_launch_arg)
    ld.add_action(verbose_launch_arg)

    vicon_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('mocap4r2_vicon_driver'), 'launch', 'mocap4r2_vicon_driver_launch.py')
        ])
    )
    ld.add_action(vicon_launch_description)


    vicon_tf_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('mocap4ros2_vicon_tf'), 'launch', 'launch.py')
        ]),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'verbose': LaunchConfiguration('verbose'),
        }.items(),
    )
    ld.add_action(vicon_tf_launch_description)

    return ld
