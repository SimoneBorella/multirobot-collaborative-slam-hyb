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

    multirobot_simulator_dir = get_package_share_directory('multirobot_simulator')

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
        multirobot_simulator_dir,
        "config",
        "params.yaml"
    )

    with open(os.path.join(get_package_share_directory('multirobot_simulator'), 'config', 'params.yaml'), "r") as file:
        mrs_config = yaml.safe_load(file)

    rviz_path = os.path.join(multirobot_simulator_dir, 'rviz', 'multirobot_simulator_view.rviz')

    # Nodes
    multirobot_simulator_node = Node(
        package="multirobot_simulator",
        executable="multirobot_simulator",
        output='screen',
        parameters=[config],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ]
    )

    ld.add_action(multirobot_simulator_node)

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'), ('/placeholder/tf', '/tf'), ('/placeholder/tf_static', '/tf_static')]

    for robot in mrs_config["multirobot_simulator"]["ros__parameters"]["robots"]:
        namespace = '/' + robot

        tf_publisher_node = Node(
            package="multirobot_simulator",
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


    visualization_publisher_node = Node(
        package="multirobot_simulator",
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

