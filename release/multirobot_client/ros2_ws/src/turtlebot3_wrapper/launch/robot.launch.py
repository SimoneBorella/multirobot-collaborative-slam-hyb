#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

def create_param_file(context, *args, **kwargs):
    turtlebot3_model = os.environ['TURTLEBOT3_MODEL']
    namespace = LaunchConfiguration('namespace').perform(context)

    input_path = os.path.join(
        get_package_share_directory('turtlebot3_wrapper'),
        'param',
        f'{turtlebot3_model}.yaml'
    )
    output_path = os.path.join(
        get_package_share_directory('turtlebot3_wrapper'),
        'param',
        f'{turtlebot3_model}_{namespace}.yaml'
    )

    with open(input_path, 'r') as input_file:
        file_content = input_file.read()

    updated_content = file_content.replace('<namespace>', namespace)

    with open(output_path, 'w') as output_file:
        output_file.write(updated_content)

    # Set this path in the launch configuration so it can be used below
    context.launch_configurations['tb3_param_dir'] = output_path

    return []

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    LDS_MODEL = os.environ['LDS_MODEL']
    CAMERA_MODEL = os.environ['CAMERA_MODEL']

    if LDS_MODEL == 'LDS-01':
        LDS_LAUNCH_FILE = '/hlds_laser.launch.py'
    elif LDS_MODEL == 'LDS-02':
        LDS_LAUNCH_FILE = '/ld08.launch.py'
    else:
        raise Exception("Unknown LDS_MODEL")

    if CAMERA_MODEL == 'oakd':
        CAMERA_LAUNCH_FILE = '/oakd.launch.py'
    else:
        raise Exception("Unknown CAMERA_MODEL")

    cam_pos_x = LaunchConfiguration("cam_pos_x", default="0.0")
    cam_pos_y = LaunchConfiguration("cam_pos_y", default="0.0")
    cam_pos_z = LaunchConfiguration("cam_pos_z", default="0.0")
    cam_roll = LaunchConfiguration("cam_roll", default="0.0")
    cam_pitch = LaunchConfiguration("cam_pitch", default="0.0")
    cam_yaw = LaunchConfiguration("cam_yaw", default="0.0")

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value='/dev/ttyACM0',
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            "cam_pos_x",
            default_value="0.0",
            description="Position X of the camera with respect to the base frame.",
        ),
        DeclareLaunchArgument(
            "cam_pos_y",
            default_value="0.0",
            description="Position Y of the camera with respect to the base frame.",
        ),
        DeclareLaunchArgument(
            "cam_pos_z",
            default_value="0.0",
            description="Position Z of the camera with respect to the base frame.",
        ),
        DeclareLaunchArgument(
            "cam_roll",
            default_value="0.0",
            description="Roll orientation of the camera with respect to the base frame.",
        ),
        DeclareLaunchArgument(
            "cam_pitch",
            default_value="0.0",
            description="Pitch orientation of the camera with respect to the base frame.",
        ),
        DeclareLaunchArgument(
            "cam_yaw",
            default_value="0.0",
            description="Yaw orientation of the camera with respect to the base frame.",
        ),

        OpaqueFunction(function=create_param_file),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), LDS_LAUNCH_FILE]),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'port': '/dev/ttyUSB0',
                'frame_id': 'base_scan'
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), CAMERA_LAUNCH_FILE]),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'parent_frame': 'base_link',
                'cam_pos_x': cam_pos_x,
                'cam_pos_y': cam_pos_y,
                'cam_pos_z': cam_pos_z,
                'cam_roll': cam_roll,
                'cam_pitch': cam_pitch,
                'cam_yaw': cam_yaw,
                'use_rviz': 'false'
            }.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('tb3_param_dir')],
            arguments=['-i', LaunchConfiguration('usb_port')],
            output='screen',
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        )
    ])
