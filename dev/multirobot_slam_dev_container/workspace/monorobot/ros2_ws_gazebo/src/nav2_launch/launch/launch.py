import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():

    ld = LaunchDescription()

    
    # Launch arguments

    robots_config_file_launch_arg = DeclareLaunchArgument(
        'robots_config_file',
        default_value=os.path.join(get_package_share_directory('launch'), 'config', 'robots.yaml')
    )

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True'
    )

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal'],
    )


    # Load robots config
    with open(os.path.join(get_package_share_directory('launch'), 'config', 'robots.yaml'), "r") as file:
        robots_config = yaml.safe_load(file)

    for robot in robots_config["robots"]:
        namespace = [ '/' + robot['name'] ]

        nav2_bringup_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_launch'), 'launch', 'bringup_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'True',
                'amcl': 'False',
                'slam': 'True',
                # 'use_namespace': 'True',
                # 'namespace': namespace,
                # 'map': os.path.join(get_package_share_directory('nav2_launch'), 'maps', 'map.yaml'),
                'map': '',
                'params_file': os.path.join(get_package_share_directory('nav2_launch'), 'params', 'nav2_params.yaml'),
                # 'params_file': os.path.join(get_package_share_directory('nav2_launch'), 'params', 'nav2_multirobot_params.yaml'),
                'log_level': LaunchConfiguration('log_level')
            }.items(),
        )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            # namespace=namespace,
            arguments=[
                '-d', os.path.join(get_package_share_directory('nav2_launch'), 'rviz', 'nav2.rviz'),
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

        ld.add_action(nav2_bringup_launch_description)
        ld.add_action(rviz_node)

    ld.add_action(robots_config_file_launch_arg)
    ld.add_action(rviz_launch_arg)
    ld.add_action(log_level_launch_arg)

    return ld
