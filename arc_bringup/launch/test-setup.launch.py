
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource

from pathlib import Path
import pathlib

CMAP = {
    "red": "\033[0;31m",
    "green": "\033[0;32m",
    "yellow": "\033[0;33m",
    "blue": "\0330;34m",
    "magenta": "\0330;35m",
    "nc": "\033[0m"
}


def green(text):
    return CMAP["green"] + text + CMAP["nc"]


def red(text):
    return CMAP["red"] + text + CMAP["nc"]


def include_yaml_launch_file(launch_file_path: Path) -> IncludeLaunchDescription:
    assert launch_file_path.exists(
    ), f"Launch file {launch_file_path} does not exist"
    return IncludeLaunchDescription(
        YAMLLaunchDescriptionSource(
            launch_file_path
        )
    )


def include_xml_launch_file(launch_file_path: Path) -> IncludeLaunchDescription:
    assert launch_file_path.exists(
    ), f"Launch file {launch_file_path} does not exist"
    return IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            launch_file_path
        )
    )


def include_python_launch_file(launch_file_path: Path) -> IncludeLaunchDescription:
    assert launch_file_path.exists(
    ), f"Launch file {launch_file_path} does not exist"
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path
        )
    )


def generate_launch_description():

    rosbridge_server = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        )
    )

    # include ds4_twist launch file
    ds4_twist = IncludeLaunchDescription(
        YAMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('arc_teleop'),
                'launch',
                'ds4_twist.launch.yaml'
            )
        )
    )
    # include vesc launch file
    vescs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('arc_bringup'),
                'launch',
                'vesc.launch.py'
            )
        )
    )

    config = os.path.join(
        get_package_share_directory('arc_bringup'),
        'config',
        'params.yaml'
    )

    # twist to diff drive
    twist_to_diff_drive = Node(
        package='arc_base',
        executable='twist_to_diff_drive_node',
        name='twist_to_diff_drive',
        parameters=[config]
    )

    print(green("config: " + config))

    # low level motor controller for left motor
    low_level_motor_controller_left = Node(
        package='arc_base',
        executable='low_level_motor_controller',
        name='low_level_motor_controller_left',
        output='screen',
        # prefix="gdb -ex run --args",
        parameters=[config],
        # parameters=[{
        #     'motor_id': 'left',
        #     'controller_rate': 10,
        #     'output_mode': 'speed',
        #     'current_max': 2.0,
        #     'speed_max': 23000.0,
        #     'acc_max': 0.1,
        #     'kp_speed': 0.004,
        #     'ki_speed': 0.004,
        #     'kd_speed': 0.0001,
        #     'kp_current': 0.004,
        #     'ki_current': 0.004,
        #     'kd_current': 0.0001,
        # }]
    )

    # low level motor controller for right motor
    low_level_motor_controller_right = Node(
        package='arc_base',
        executable='low_level_motor_controller',
        name='low_level_motor_controller_right',
        parameters=[config],
        # parameters=[{
        #     'motor_id': 'right',
        #     'controller_rate': 10,
        #     'output_mode': 'speed',
        #     'current_max': 2.0,
        #     'speed_max': 23000.0,
        #     'acc_max': 0.1,
        #     'kp_speed': 0.004,
        #     'ki_speed': 0.004,
        #     'kd_speed': 0.0001,
        #     'kp_current': 0.004,
        #     'ki_current': 0.004,
        #     'kd_current': 0.0001,
        # }]
    )

    return LaunchDescription([
        # rosbridge_server_launch,
        # ds4_twist_launch,
        # vesc_launch,
        rosbridge_server,
        ds4_twist,
        vescs,
        twist_to_diff_drive,
        low_level_motor_controller_right,
        low_level_motor_controller_left,
    ])
