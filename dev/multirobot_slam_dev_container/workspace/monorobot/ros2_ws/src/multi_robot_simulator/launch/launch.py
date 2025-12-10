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

    multi_robot_simulator_dir = get_package_share_directory('multi_robot_simulator')

    # Launch arguments

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True'
    )

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )

    ld.add_action(rviz_launch_arg)
    ld.add_action(log_level_launch_arg)

    # Configuration file

    config = os.path.join(
        multi_robot_simulator_dir,
        "config",
        "params.yaml"
    )

    with open(os.path.join(get_package_share_directory('multi_robot_simulator'), 'config', 'params.yaml'), "r") as file:
        mrs_config = yaml.safe_load(file)

    rviz_path = os.path.join(multi_robot_simulator_dir, 'rviz', 'multi_robot_simulator_view.rviz')

    # Nodes
    multi_robot_simulator_node = Node(
        package="multi_robot_simulator",
        executable="multi_robot_simulator",
        output='screen',
        parameters=[config],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ]
    )

    ld.add_action(multi_robot_simulator_node)

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'), ('/placeholder/tf', '/tf'), ('/placeholder/tf_static', '/tf_static')]

    for robot in mrs_config["multi_robot_simulator"]["ros__parameters"]["robots"]:
        namespace = '/' + robot

        tf_publisher_node = Node(
            package="multi_robot_simulator",
            executable="tf_publisher",
            namespace=namespace,
            output='screen',
            parameters=[config],
            arguments=[
                '--ros-args', '--log-level', LaunchConfiguration('log_level')
            ],
            remappings=remappings
        )

        ld.add_action(tf_publisher_node)


    # clock_publisher_node = Node(
    #     package="multi_robot_simulator",
    #     executable="clock_publisher",
    #     output='screen',
    #     parameters=[config],
    #     arguments=[
    #         '--ros-args', '--log-level', LaunchConfiguration('log_level')
    #     ]
    # )
    #
    # ld.add_action(clock_publisher_node)

    visualization_publisher_node = Node(
        package="multi_robot_simulator",
        executable="visualization_publisher",
        output='screen',
        parameters=[config],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ]
    )

    ld.add_action(visualization_publisher_node)

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
    
    return ld

