from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

import os


def expand(path: str) -> str:
    expanded_path = os.path.expanduser(path)
    # assert that the path exists
    assert os.path.exists(expanded_path), f"{expanded_path} does NOT exist!"
    return expanded_path


# ros2 run tf2_ros static_transform_publisher 1 2 3 0.5 0.1 -1.0 foo bar

def gen_static_transform_publisher(x: float, y: float, z: float, qx: float, qy: float, qz: float, q0: float, frame_id: str, child_frame_id: str) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[str(x), str(y), str(z), str(qx), str(
            qy), str(qz), str(q0), frame_id, child_frame_id],
        output='screen'
    )


def generate_launch_description() -> LaunchDescription:
    rectify = 'true'
    equalize = 'true'
    orbslam3 = Node(
        package="orbslam3",
        executable="stereo-inertial",
        output='screen',
        arguments=[
            # TODO: do not hardcode abs path
            expand(
                '~/multi-agent-robotics/ORB-SLAM3-STEREO-FIXED/vocabulary/ORBvoc.txt'),
            expand('~/ros2_ws/src/orbslam3/orbslam3/config/stereo-inertial/EuRoC.yaml'),
            rectify,
            equalize
        ]
    )

    path_to_euroc_dataset_bag = expand('~/rosbags/EuRoC/MH_01_easy/')

    # old_topic:=new_topic
    cmd: str = f"ros2 bag play {path_to_euroc_dataset_bag} --remap /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu"

    play_euroc_dataset = ExecuteProcess(
        cmd=cmd.split(),
        # output='screen'
    )

    # # TODO: do not hardcode abs path
    # path_to_rviz2_config = expand('~/ros2_ws/src/rnd-lugbot/rviz2/orbslam3-ros2-euroc-dataset.rviz')
    # rviz2 = Node(
    #     package='rviz2',
    #     namespace='',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', path_to_rviz2_config],
    # )

    # starts a websocket server on port 9090
    # used by Foxglove Studio to visualize the data
    rosbridge_websocker_server = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server',
             'rosbridge_websocket_launch.xml'],
    )

    return LaunchDescription([
        gen_static_transform_publisher(0, 0, 0, 0, 0, 0, 1, 'map', 'camera'),
        play_euroc_dataset,
        orbslam3,
        #        rviz2,
        rosbridge_websocker_server
    ])


OnShutdown(on_shutdown=[LogInfo(msg=['Launch file is shutting down'])])
