from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    # Launch arguments

    x_init_launch_arg = DeclareLaunchArgument(
        'x_init',
        default_value='0.0'
    )
    y_init_launch_arg = DeclareLaunchArgument(
        'y_init',
        default_value='0.0'
    )
    z_init_launch_arg = DeclareLaunchArgument(
        'z_init',
        default_value='0.0'
    )
    R_init_launch_arg = DeclareLaunchArgument(
        'R_init',
        default_value='0.0'
    )
    P_init_launch_arg = DeclareLaunchArgument(
        'P_init',
        default_value='0.0'
    )
    Y_init_launch_arg = DeclareLaunchArgument(
        'Y_init',
        default_value='0.0'
    )

    pose_cov_init_launch_arg = DeclareLaunchArgument(
        'pose_cov_init',
        default_value='[ \
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0, \
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0, \
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0, \
            0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, \
            0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, \
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0001 \
        ]',
    )

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )

    # Nodes

    initial_pose_publisher_node = Node(
        package='bt_navigation',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        parameters=[{
            'x_init': LaunchConfiguration('x_init'),
            'y_init': LaunchConfiguration('y_init'),
            'z_init': LaunchConfiguration('z_init'),
            'R_init': LaunchConfiguration('R_init'),
            'P_init': LaunchConfiguration('P_init'),
            'Y_init': LaunchConfiguration('Y_init'),
            'pose_cov_init': LaunchConfiguration('pose_cov_init')
        }],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ]
    )

    bt_navigation_node = Node(
        package='bt_navigation',
        executable='bt_navigation',
        name='bt_navigation',
        parameters=[{
            
        }],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ]
    )

    return LaunchDescription([
        # Launch arguments
        x_init_launch_arg,
        y_init_launch_arg,
        z_init_launch_arg,
        R_init_launch_arg,
        P_init_launch_arg,
        Y_init_launch_arg,
        pose_cov_init_launch_arg,
        log_level_launch_arg,

        # Nodes
        initial_pose_publisher_node,
        bt_navigation_node
    ])