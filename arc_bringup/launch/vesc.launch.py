import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    vesc_left_config = os.path.join(
        get_package_share_directory('arc_bringup'),
        'config',
        'vesc-left.yaml'
    )

    print(vesc_left_config)

    vesc_right_config = os.path.join(
        get_package_share_directory('arc_bringup'),
        'config',
        'vesc-right.yaml'
    )

    print(vesc_right_config)
    # vesc_config = os.path.join(
    #    get_package_share_directory('vesc_driver'),
    #    'params',
    #    'vesc_config.yaml'
    #    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name="config-left",
            default_value=vesc_left_config,
            description="VESC yaml configuration file.",
        ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            namespace='motor_left',
            name='vesc_driver_node',
            parameters=[LaunchConfiguration("config-left")]
        ),
        DeclareLaunchArgument(
            name="config-right",
            default_value=vesc_right_config,
            description="VESC yaml configuration file.",
        ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            namespace='motor_right',
            name='vesc_driver_node',
            parameters=[LaunchConfiguration("config-right")]
        )
    ])
