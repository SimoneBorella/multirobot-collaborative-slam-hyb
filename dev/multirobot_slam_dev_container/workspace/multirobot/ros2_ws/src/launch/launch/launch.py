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
        '/costmap',
        '/costmap_updates',
        '/frontier',
        '/frontier_centroids',
        '/frontier_centroids_array',
        '/frontier_updates',
        '/goal_pose',
        '/initialpose',
        '/map',
        '/map_updates',
        '/tf',
        '/tf_static'
    ]


    with open(os.path.join(get_package_share_directory('slam'), 'config', 'params.yaml'), "r") as file:
        mrs_config = yaml.safe_load(file)


    for robot_name in mrs_config["slam"]["ros__parameters"]["robots"]:
        topics_to_record += [
            f'/{robot_name}/cmd_vel',
            f'/{robot_name}/footprint',
            f'/{robot_name}/footprint_array',
            f'/{robot_name}/global_path',
            f'/{robot_name}/imu',
            f'/{robot_name}/joint_states',
            f'/{robot_name}/landmarks',
            f'/{robot_name}/landmarks_marker',
            f'/{robot_name}/landmarks_plot',
            f'/{robot_name}/landmarks_plot_array',
            # f'/{robot_name}/oak/imu/data',
            # f'/{robot_name}/oak/nn/spatial_detections',
            # f'/{robot_name}/oak/rgb/camera_info',
            # f'/{robot_name}/oak/rgb/image_raw',
            # f'/{robot_name}/oak/rgb/image_raw/compressed',
            # f'/{robot_name}/oak/rgb/image_raw/compressedDepth',
            # f'/{robot_name}/oak/rgb/image_raw/theora',
            # f'/{robot_name}/oak/rgb/image_rect',
            # f'/{robot_name}/oak/rgb/image_rect/compressed',
            # f'/{robot_name}/oak/rgb/image_rect/compressedDepth',
            # f'/{robot_name}/oak/rgb/image_rect/theora',
            # f'/{robot_name}/oak/rgb_landmarks/image_raw',
            # f'/{robot_name}/oak/rgb_landmarks/image_raw/compressed',
            # f'/{robot_name}/oak/rgb_landmarks/image_raw/compressedDepth',
            # f'/{robot_name}/oak/rgb_landmarks/image_raw/theora',
            # f'/{robot_name}/oak/stereo/camera_info',
            # f'/{robot_name}/oak/stereo/image_raw',
            # f'/{robot_name}/oak/stereo/image_raw/compressed',
            # f'/{robot_name}/oak/stereo/image_raw/compressedDepth',
            # f'/{robot_name}/oak/stereo/image_raw/theora',
            f'/{robot_name}/odom',
            f'/{robot_name}/robot_description',
            f'/{robot_name}/scan',
            f'/{robot_name}/scan_plot',
            f'/{robot_name}/sensor_state',
            f'/{robot_name}/map',
            f'/{robot_name}/tf',
            f'/{robot_name}/tf_static',
        ]

    bag_record_execute_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_name] + topics_to_record,
        output='screen',
        condition=IfCondition(LaunchConfiguration('bag_record')),
    )

    ld.add_action(bag_record_execute_process)






    # Launch descriptions

    multi_robot_simulator_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('multi_robot_simulator'), 'launch','launch.py')
        ]),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'log_level': LaunchConfiguration('log_level'),
        }.items()
    )

    ld.add_action(multi_robot_simulator_launch_description)


    slam_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam'), 'launch', 'launch.py')
        ]),
        launch_arguments={
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )    

    ld.add_action(slam_launch_description)
        
    return ld

