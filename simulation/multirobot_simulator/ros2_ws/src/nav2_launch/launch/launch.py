import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Get the launch directory
    nav2_launch_dir = get_package_share_directory('nav2_launch')
    launch_dir = os.path.join(nav2_launch_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    slam_toolbox = LaunchConfiguration('slam_toolbox')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    log_level = LaunchConfiguration('log_level')

    use_localization = slam_toolbox


    # Declare the launch arguments

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace',
    )

    declare_slam_toolbox_cmd = DeclareLaunchArgument(
        'slam_toolbox',
        default_value='True',
        description='Use slam_toolbox if true',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock if true',
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='True',
        description='Whether to start RVIZ'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level'
    )


    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_toolbox_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_log_level_cmd)


    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_localization': use_localization,
            'slam': slam_toolbox,
            'map': '',
            # 'map': os.path.join(nav2_launch_dir, 'maps', 'map.yaml'),
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(nav2_launch_dir, 'params', 'nav2_params.yaml'),
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': log_level,
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(rviz),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'rviz_config': os.path.join(nav2_launch_dir, 'rviz', 'nav2_view.rviz'),
            'log_level': log_level,
        }.items(),
    )

    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld