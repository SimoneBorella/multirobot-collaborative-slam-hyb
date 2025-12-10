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

    mocap4ros2_vicon_tf_dir = get_package_share_directory('mocap4ros2_vicon_tf')

    # Launch arguments

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False'
    )

    verbose_launch_arg = DeclareLaunchArgument(
        'verbose',
        default_value='False'
    )

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )

    ld.add_action(rviz_launch_arg)
    ld.add_action(verbose_launch_arg)
    ld.add_action(log_level_launch_arg)

    # Configuration file
    config = os.path.join(
        mocap4ros2_vicon_tf_dir,
        "config",
        "params.yaml"
    )

    rviz_path = os.path.join(mocap4ros2_vicon_tf_dir, 'rviz', 'vicon_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', rviz_path,
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    LaunchConfiguration('rviz'), " == True",
                ]
            )
        )
    )

    ld.add_action(rviz_node)

    tf_publisher_node = Node(
        package="mocap4ros2_vicon_tf",
        executable="tf_publisher",
        output='screen',
        parameters=[
            config,
            {"verbose": LaunchConfiguration('verbose')}
        ],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ]
    )

    ld.add_action(tf_publisher_node)
    
    return ld

