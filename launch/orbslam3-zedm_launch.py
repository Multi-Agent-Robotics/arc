from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


import os

expand = lambda path: os.path.expanduser(path)

def generate_launch_description() -> LaunchDescription:

    rectify = 'true'
    equalize = 'true'
    orbslam3 = Node(
        package="orbslam3",
        executable="stereo-inertial",
        output='screen',
        arguments=[
            expand('~/ros2_ws/src/orbslam3/vocabulary/ORBvoc.txt'),
            expand('~/ros2_ws/src/orbslam3/config/stereo-inertial/EuRoC.yaml'), 
            rectify,
            equalize
        ]
    )

    # IncludeLaunchDescription(
    #     package='orbslam3',
    #     launch='stereo-inertial',
    #     arguments=['~/ros2_ws/src/orbslam3/vocabulary/ORBvoc.txt',
    #                '~/ros2_ws/src/orbslam3/config/stereo-inertial/EuRoC.yaml', 'true']
    # ),

    zedm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("zed_wrapper"), '/launch', '/zedm.launch.py']),
    )

    # zedm = IncludeLaunchDescription(
    #     package='zed_wrapper',
    #     launch='zedm.launch.py',
    #     arguments=[''],
    #     remappings=[
    #             ('/zedm/zed_node/left/image_rect_color', '/camera/left'),
    #             ('/zedm/zed_node/right/image_rect_color', '/camera/right'),
    #             ('/zedm/zed_node/imu/data', '/imu')
    #     ]
    # ),

    return LaunchDescription([
        GroupAction(actions=[
            SetRemap(src='/zedm/zed_node/left_raw/image_raw_color', dst='/camera/left'),
            SetRemap(src='/zedm/zed_node/right_raw/image_raw_color', dst='/camera/right'),
            SetRemap(src='/zedm/zed_node/imu/data', dst='/imu'),
            zedm
        ]),
        orbslam3
        
    ])


OnShutdown(on_shutdown=[LogInfo(msg=['Launch file is shutting down'])])