from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('camera_name', default_value='camera'), # Dummy argument
    ]

    # Launch description
    ld = LaunchDescription(
        args 
        + [
            
            # https://github.com/edgarwelteKIT/robotiq_2f_urcap_adapter
            Node(
                package="robotiq_2f_urcap_adapter",
                executable="robotiq_2f_mock_server.py",
                name="robotiq_2f_mock_server",
                output="screen",
                arguments=["63353"],
            ),

            # https://github.com/edgarwelteKIT/robotiq_2f_urcap_adapter
            Node(
                package="robotiq_2f_urcap_adapter",
                executable="robotiq_2f_adapter_node.py",
                name="robotiq_2f_urcap_adapter",
                output="screen",
                parameters = [{
                    "robot_ip": "127.0.0.1",
                    "robot_port": 63353,
                }],
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("ifl_air_arm_api_launch"), 
                        'launch',
                        'ur10e.launch.py'
                    ])
                ]),
                launch_arguments={
                    'launch_rviz': 'true',
                    'ur_type': 'ur10e',
                    'description_package': 'ifl_air_arm_api_launch',
                    'description_file': 'robot_ur_robotiq2f85.urdf.xacro',
                    'robot_ip': '192.168.56.101'
                }.items()
            )
        ]
    )
    return ld
