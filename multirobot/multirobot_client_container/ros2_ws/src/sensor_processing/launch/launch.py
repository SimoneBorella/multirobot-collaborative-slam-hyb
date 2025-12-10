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

    sensor_processing_dir = get_package_share_directory('sensor_processing')

    # Launch arguments

    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace',
    )

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )

    # Configuration file

    config = os.path.join(
        sensor_processing_dir,
        "config",
        "params.yaml"
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=config,
            root_key=LaunchConfiguration('namespace'),
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Nodes
    rgbd_feature_extraction_node = Node(
        package="sensor_processing",
        executable="rgbd_feature_extraction",
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[configured_params],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ],
        remappings=remappings
    )


    # Launch arguments
    ld.add_action(namespace_launch_arg)
    ld.add_action(log_level_launch_arg)

    # Nodes
    ld.add_action(rgbd_feature_extraction_node)
    
    return ld

