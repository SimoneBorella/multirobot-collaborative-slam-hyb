import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterFile

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    multirobot_server_dir = get_package_share_directory('multirobot_server')

    # Launch arguments

    # rviz_launch_arg = DeclareLaunchArgument(
    #     'rviz',
    #     default_value='True'
    # )

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )

    # ld.add_action(rviz_launch_arg)
    ld.add_action(log_level_launch_arg)

    # Configuration file

    config = os.path.join(
        multirobot_server_dir,
        "config",
        "params.yaml"
    )

    # rviz_path = os.path.join(multirobot_server_dir, 'rviz', 'multirobot_server_view.rviz')

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


    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=[
    #         '-d', rviz_path,
    #         '--ros-args', '--log-level', LaunchConfiguration('log_level')
    #     ],
    #     condition=IfCondition(
    #         PythonExpression(
    #             [
    #                 LaunchConfiguration('rviz'), " == True",
    #             ]
    #         )
    #     )
    # )

    # ld.add_action(rviz_node)
    
    return ld

