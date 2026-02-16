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

    launch_dir = get_package_share_directory('launch')


    # Launch arguments

    bag_record_launch_arg = DeclareLaunchArgument(
        'bag_record',
        default_value='False',
        description='Enable ros2 bag recording'
    )

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True'
    )

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )

    ld.add_action(bag_record_launch_arg)
    ld.add_action(rviz_launch_arg)
    ld.add_action(log_level_launch_arg)


    timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    bag_name = f'./bag_records/bag_{timestamp}'


    topics_to_record = [
        '/map',
        '/map_ground_truth',
        '/frontiers_marker',
        '/refinement_frontiers_marker',
        '/tf',
        '/tf_static'
    ]


    with open(os.path.join(get_package_share_directory('multirobot_server'), 'config', 'params.yaml'), "r") as file:
        mrs_config = yaml.safe_load(file)


    for robot_name in mrs_config["multirobot_server"]["ros__parameters"]["robots"]:
        topics_to_record += [
            f'/{robot_name}/cmd_vel',
            f'/{robot_name}/global_path',
            f'/{robot_name}/odom',
            f'/{robot_name}/state',
            f'/{robot_name}/map',
            f'/{robot_name}/keyframes_update',
            f'/{robot_name}/keyframes_marker',
            f'/{robot_name}/tf',
            f'/{robot_name}/tf_static',
        ]

    bag_record_execute_process = ExecuteProcess(
        # Record with MCAP
        # cmd=['ros2', 'bag', 'record', '-o', bag_name] + topics_to_record,
        # Record with Sqlite3
        cmd=['ros2', 'bag', 'record', '-s', 'sqlite3', '-o', bag_name] + topics_to_record,
        output='screen',
        condition=IfCondition(LaunchConfiguration('bag_record')),
    )

    ld.add_action(bag_record_execute_process)






    # Launch descriptions

    # Launch simulator
    multirobot_simulator_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('multirobot_simulator'), 'launch','launch.py')
        ]),
        launch_arguments={
            'log_level': LaunchConfiguration('log_level'),
        }.items()
    )

    ld.add_action(multirobot_simulator_launch_description)

    # Launch multirobot server
    multirobot_server_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('multirobot_server'), 'launch', 'launch.py')
        ]),
        launch_arguments={
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )    

    ld.add_action(multirobot_server_launch_description)

    # Launch multirobot client for each robot
    for robot_name in mrs_config["multirobot_server"]["ros__parameters"]["robots"]:
        multirobot_client_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('multirobot_client'), 'launch', 'launch.py')
            ]),
            launch_arguments={
                'namespace': robot_name,
                'log_level': LaunchConfiguration('log_level'),
            }.items(),
        )
        ld.add_action(multirobot_client_launch_description)


    rviz_path = os.path.join(launch_dir, 'rviz', 'view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', rviz_path,
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('rviz'), "' == 'True'"
            ])
        )
    )

    ld.add_action(rviz_node)

    return ld

