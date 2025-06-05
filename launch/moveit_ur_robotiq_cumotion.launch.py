# This launch file is created following the tutorial at 
# https://moveit.picknik.ai/main/doc/how_to_guides/moveit_launch_files/moveit_launch_files_tutorial.html
from os import path
from ament_index_python.packages import get_package_share_directory

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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
            pipelines=["isaac_ros_cumotion","ompl","pilz_industrial_motion_planner","chomp"]
        ) # optional: can be omitted to use the default pipeline ("stomp" is currently not supported in humble)
        .sensors_3d(file_path=None)
        .to_moveit_configs()
    )

    publish_robot_description_semantic = {"publish_robot_description_semantic": True}
    publish_robot_description = {"publish_robot_description": True}
    publish_robot_description_kinematics = {"publish_robot_description_kinematics": True}
    publish_robot_description_kinematics = {"publish_robot_description_kinematics": True}
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    publish_robot_description,
                    publish_robot_description_kinematics,
                    publish_robot_description_semantic,
                    planning_scene_monitor_parameters],
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

    # cumotion node
    
    xacro_file = PathJoinSubstitution([
                    FindPackageShare("ifl_air_arm_api_launch"), 
                    "urdf/robot_ur_robotiq2f85.urdf.xacro"
                ])
    urdf_file = PathJoinSubstitution([
                    FindPackageShare("ifl_air_arm_api_moveit_config"),
                    "urdf/robot_ur_robotiq2f85.urdf"
                ])
    
    # Convert Xacro to URDF (dynamically at launch time to ensure the latest version is used)
    xacro_to_urdf = ExecuteProcess(
        cmd=['xacro', xacro_file, '-o', urdf_file],
        output='screen'
    )
    
    
    cumotion_node = Node(
        package="isaac_ros_cumotion",
        executable="cumotion_planner_node",
        output="screen",
        parameters=[
            {"robot": PathJoinSubstitution([
                FindPackageShare("ifl_air_arm_api_moveit_config"),
                "xrdf/robot_ur_robotiq2f85.xrdf"
            ])},
            {"urdf_path": urdf_file},
        ],
    )
    
    # Ensure that robot_state_publisher starts only after xacro_to_urdf finishes
    launch_cumotion_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=xacro_to_urdf,
            on_exit=[cumotion_node]
        )
    )
    
    # scene loader node
    #scene_loader_node = TimerAction(
    #    period=1.0,
    #    actions=[
    #        Node(
    #            package="cumotion_moveit_scene_loader",
    #            executable="cumotion_moveit_scene_loader.py",
    #            output="screen",
    #        )
    #    ]
    #)


    return LaunchDescription([
        rviz_config_arg,
        launch_joy_arg,
        run_move_group_node,
        rviz_node,
        moveit_wrapper,
        xacro_to_urdf,
        launch_cumotion_node,
        #scene_loader_node,
    ])
