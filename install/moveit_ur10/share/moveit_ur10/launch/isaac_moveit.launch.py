#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Command-line arguments
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )
    
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    ) 
    
    isaacsim_share = get_package_share_directory('isaacsim')
    planner_share = get_package_share_directory('moveit_ur10')
     # IsaacSim launch
    isaacsim_launch_file = os.path.join(
        isaacsim_share, 'launch', 'run_isaacsim.launch.py'
    )
    usd_path = os.path.join(planner_share, 'sim', 'test.usd') #itobos_env 

    isaacsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(isaacsim_launch_file),
        launch_arguments={
            'gui': usd_path,
            'play_sim_on_start': 'true'
        }.items()
    )
    
    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("ur10_with_gripper", package_name="moveit_ur10")
        .robot_description(
            file_path="config/ur10_with_gripper_isaac.urdf.xacro",
            mappings={
                "ros2_control_hardware_type":  LaunchConfiguration(
                    "ros2_control_hardware_type"
                ),
            },
        )
        .robot_description_semantic(file_path="config/ur10_with_gripper.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    # MoveIt node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
            {"publish_planning_scene": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )


    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # ROS2 Controllers path
    ros2_controllers_path = PathJoinSubstitution(
        [FindPackageShare("moveit_ur10"), "config", "ros2_controllers.yaml"]
    )
    
    # # Generate robot description directly
    # robot_description_content = Command([
    #     "xacro", " ",
    #     PathJoinSubstitution([FindPackageShare("moveit_ur10"), "config", "ur10_with_gripper.urdf.xacro"]),
    #     " initial_positions_file:=",
    #     PathJoinSubstitution([FindPackageShare("moveit_ur10"), "config", "initial_positions.yaml"]),
    # ])
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )


    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("moveit_ur10"), "config", "moveit.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(
        [   
            # Start IsaacSim first - this takes time to initialize
            isaacsim_launch,
            ros2_control_hardware_type,
            use_sim_time,
            
            # Start basic ROS2 nodes immediately (these don't depend on IsaacSim)
            static_tf,
            robot_state_publisher,
            
            # Wait 15 seconds for IsaacSim to initialize, then start control nodes
            TimerAction(
                period=15.0,
                actions=[
                    ros2_control_node,
                    joint_state_broadcaster_spawner,
                    arm_controller_spawner,
                    gripper_controller_spawner,
                ]
            ),
            
            # Wait 20 seconds total before starting MoveIt (needs robot state)
            TimerAction(
                period=20.0,
                actions=[move_group_node]
            ),
            
            # Start RViz last (25 seconds) to ensure everything is ready
            TimerAction(
                period=25.0,
                actions=[rviz_node]
            ),
        ]
    )