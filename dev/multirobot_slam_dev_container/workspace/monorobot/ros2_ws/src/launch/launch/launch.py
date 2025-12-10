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

    rviz_simulator_launch_arg = DeclareLaunchArgument(
        'rviz_simulator',
        default_value='True'
    )

    rviz_nav2_launch_arg = DeclareLaunchArgument(
        'rviz_nav2',
        default_value='False'
    )

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )


    ld.add_action(rviz_simulator_launch_arg)
    ld.add_action(rviz_nav2_launch_arg)
    ld.add_action(log_level_launch_arg)



    # Launch descriptions

    multi_robot_simulator_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('multi_robot_simulator'), 'launch','launch.py')
        ]),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz_simulator'),
            'log_level': LaunchConfiguration('log_level'),
        }.items()
    )

    ld.add_action(multi_robot_simulator_launch_description)


    # Load robots config
    with open(os.path.join(get_package_share_directory('multi_robot_simulator'), 'config', 'params.yaml'), "r") as file:
        mrs_config = yaml.safe_load(file)

    for robot in mrs_config["multi_robot_simulator"]["ros__parameters"]["robots"]:
        namespace = '/' + robot

        nav2_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_launch'), 'launch', 'launch.py')
            ]),
            launch_arguments={
                'namespace': namespace,
                'slam_toolbox': 'False',
                'rviz': LaunchConfiguration('rviz_nav2'),
                'log_level': LaunchConfiguration('log_level'),
            }.items(),
        )

        slam_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam'), 'launch', 'launch.py')
            ]),
            launch_arguments={
                'namespace': namespace,
                'log_level': LaunchConfiguration('log_level'),
            }.items(),
        )

        ld.add_action(nav2_launch_description)
        ld.add_action(slam_launch_description)
        
    return ld

