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
            # expand('~/ros2_ws/src/orbslam3/vocabulary/ORBvoc.txt'),
            expand('~/multi-agent-robotics/ORB-SLAM3-STEREO-FIXED/vocabulary/ORBvoc.txt'),
            expand('~/ros2_ws/src/orbslam3/config/stereo-inertial/EuRoC.yaml'), 
            rectify,
            equalize
        ]
    )

    zedm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("zed_wrapper"), '/launch', '/zedm.launch.py']),
    )

    zedm_with_remapped_topics = GroupAction(actions=[
        SetRemap(src='/zedm/zed_node/left_raw/image_raw_color', dst='/camera/left'),
        SetRemap(src='/zedm/zed_node/right_raw/image_raw_color', dst='/camera/right'),
        SetRemap(src='/zedm/zed_node/imu/data', dst='/imu'),
        zedm
    ])

    return LaunchDescription([
        # GroupAction(actions=[
        #     SetRemap(src='/zedm/zed_node/left_raw/image_raw_color', dst='/camera/left'),
        #     SetRemap(src='/zedm/zed_node/right_raw/image_raw_color', dst='/camera/right'),
        #     SetRemap(src='/zedm/zed_node/imu/data', dst='/imu'),
        #     zedm
        # ]),
        zedm_with_remapped_topics,
        orbslam3
        
    ])


OnShutdown(on_shutdown=[LogInfo(msg=['Launch file is shutting down'])])