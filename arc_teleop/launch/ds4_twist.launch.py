from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument




def generate_launch_description() -> LaunchDescription:

    addr = DeclareLaunchArgument('addr', default_value="")
    stamped = DeclareLaunchArgument('stamped', default_value="false")
    use_standard_msgs = DeclareLaunchArgument('use_standard_msgs', default_value="false")

    ds4_driver = IncludeLaunchDescription(
        FindPackageShare('ds4_driver') / 'launch' / 'ds4_driver.launch.xml',
        launch_arguments=[
            addr,
            use_standard_msgs,
        ]
    )

    ds4_twist = Node(
        package='ds4_driver',
        exec_name='ds4_twist_node.py',
        name='ds4_twist',
        parameters=[{
            "stamped": stamped
        }]
    )

    return LaunchDescription([
        ds4_driver,
        ds4_twist
    ])