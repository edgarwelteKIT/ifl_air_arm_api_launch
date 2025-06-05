# This launch file is created following the tutorial at 
# https://moveit.picknik.ai/main/doc/how_to_guides/moveit_launch_files/moveit_launch_files_tutorial.html

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():

    # define xacro mapping for the robot description file

    launch_arguments ={
        "ur_type": "ur10e",

    }  

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "air_ur_robotiq", package_name="ifl_air_arm_api_moveit_config"
        )
        .robot_description(mappings=launch_arguments) # optional: can be omitted to use default arguments
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"]
        ) # optional: can be omitted to use the default pipeline ("stomp" is currently not supported in humble)
        .sensors_3d(file_path=None)
        .to_moveit_configs()
    )

    publish_robot_description_semantic = {"publish_robot_description_semantic": True}
    publish_robot_description = {"publish_robot_description": True}
    publish_robot_description_kinematics = {"publish_robot_description_kinematics": True}

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    publish_robot_description,
                    publish_robot_description_kinematics,
                    publish_robot_description_semantic, ],
    )

    # RViz for visualization
    # Get the path to the RViz config file
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config", 
        default_value=str(moveit_config.package_path / "config/moveit.rviz"), 
        description="RViz config file"
    )
    
    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    launch_joy_arg = DeclareLaunchArgument('launch_joy', default_value='false')

    # arm_api2 moveit wrapper
    moveit_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("arm_api2"), 
                'launch',
                'moveit2_iface.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': 'ur',
            'launch_joy': LaunchConfiguration("launch_joy"),
        }.items()
    )



    return LaunchDescription([
        rviz_config_arg,
        launch_joy_arg,
        run_move_group_node,
        rviz_node,
        moveit_wrapper,
    ])
