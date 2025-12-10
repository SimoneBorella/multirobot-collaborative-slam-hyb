import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


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


    # Environment variables

    gz_resource_path_env = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(get_package_share_directory('ros_gz_description'), 'models'))


    # Load robots config
    with open(os.path.join(get_package_share_directory('launch'), 'config', 'robots.yaml'), "r") as file:
        robots_config = yaml.safe_load(file)

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Launch descriptions

    ros_gz_sim_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                get_package_share_directory('ros_gz_gazebo'), 'worlds', 'turtlebot3_world.sdf'
            ]),
            'on_exit_shutdown': 'True',
            'log_level': LaunchConfiguration('log_level'),
        }.items()
    )


    # Nodes

    last_action = None

    for robot in robots_config["robots"]:
        namespace = [ '/' + robot['name'] ]


        # with open(os.path.join(get_package_share_directory('ros_gz_description'), 'models', robot["type"], 'model.sdf'), 'r') as input_file:
        #     file_content = input_file.read()

        # updated_content = file_content.replace("%namespace%", namespace[0])

        # with open(os.path.join(get_package_share_directory('ros_gz_description'), 'models', robot["type"], f'{robot["name"]}.sdf'), 'w') as output_file:
        #     output_file.write(updated_content)


        # with open(os.path.join(get_package_share_directory('ros_gz_description'), 'models', robot["type"], 'model.config'), 'r') as input_file:
        #     file_content = input_file.read()

        # updated_content = file_content.replace("model.sdf", f'{robot["name"]}.sdf')

        # with open(os.path.join(get_package_share_directory('ros_gz_description'), 'models', robot["type"], f'{robot["name"]}.config'), 'w') as output_file:
        #     output_file.write(updated_content)






        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            # namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                # 'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('ros_gz_description'), 'models', robot["type"], f'{robot["name"]}.sdf')])
                'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('ros_gz_description'), 'models', robot["type"], 'model.sdf')])
            }],
            arguments=[
                '--ros-args', '--log-level', LaunchConfiguration('log_level'),
            ],
            remappings=remappings,
        )

        create_robot_node = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                # '-name', robot["name"],
                # '-file', os.path.join(get_package_share_directory('ros_gz_description'), 'models', robot["type"], f'{robot["name"]}.sdf'),
                '-file', os.path.join(get_package_share_directory('ros_gz_description'), 'models', robot["type"], 'model.sdf'),
                '-x',  str(robot["x_init"]),
                '-y',  str(robot["y_init"]),
                '-z',  str(robot["z_init"]),
                '-R',  str(robot["R_init"]),
                '-P',  str(robot["P_init"]),
                '-Y',  str(robot["Y_init"]),
                '--ros-args', '--log-level', LaunchConfiguration('log_level')
            ]
        )


        if last_action is None:
            ld.add_action(create_robot_node)
            ld.add_action(robot_state_publisher_node)

        else:
            spawn_robot_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[
                        create_robot_node,
                        robot_state_publisher_node
                    ],
                )
            )

            ld.add_action(spawn_robot_event)

        last_action = create_robot_node


    ros_gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory('ros_gz_bringup'), 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', os.path.join(get_package_share_directory('ros_gz_bringup'), 'rviz', 'gazebo.rviz'),
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

    

    # Environment variables
    ld.add_action(gz_resource_path_env)

    # Launch arguments
    ld.add_action(robots_config_file_launch_arg)
    ld.add_action(rviz_launch_arg)
    ld.add_action(log_level_launch_arg)

    # Launch descriptions
    ld.add_action(ros_gz_sim_launch_description)
        

    ld.add_action(ros_gazebo_bridge_node)
    ld.add_action(rviz_node)

    return ld

