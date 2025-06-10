#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction

def generate_launch_description():
    """
    Launch setup function with context access for dynamic configuration
    """
    
    # ================================================================
    # INITIALIZE LAUNCH CONFIGURATIONS WITH CONTEXT
    # ================================================================
    robot_ip = "192.168.1.102"
    ur_type = "ur5e" 
    safety_limits = "true"
    headless_mode = "false"
    use_tool_communication = "false"

    # ================================================================
    # DYNAMIC CONFIGURATION WITH CONTEXT
    # ================================================================
    
    
    # UR update rate configuration
    ur5e_update_rate_config = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),
        "config", 
        "ur5e_update_rate.yaml"
    ])

    # ================================================================
    # ROBOT DESCRIPTION GENERATION
    # ================================================================
    
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("inspection_cell_description"), 
            "urdf", 
            "inspection_cell.urdf.xacro"
        ]),
        " ",
        "name:=inspection_cell",
        " ",
        "ur_type:=", ur_type,
        " ",
        "use_fake_hardware:=false",  # CRITICAL: Real hardware mode
        " ",
        "robot_ip:=", robot_ip,
        " ",
        "safety_limits:=", safety_limits,
        " ",
        "headless_mode:=", headless_mode,
        " ",
        "use_tool_communication:=", use_tool_communication,
        " ",
        # UR Driver Required Files
        "script_filename:=",
        PathJoinSubstitution([
            FindPackageShare("ur_client_library"), 
            "resources", 
            "external_control.urscript"
        ]),
        " ",
        "output_recipe_filename:=",
        PathJoinSubstitution([
            FindPackageShare("ur_robot_driver"), 
            "resources", 
            "rtde_output_recipe.txt"
        ]),
        " ",
        "input_recipe_filename:=",
        PathJoinSubstitution([
            FindPackageShare("ur_robot_driver"), 
            "resources", 
            "rtde_input_recipe.txt"
        ]),
        " ",
        # System Configuration Files
        "joint_limit_params:=",
        PathJoinSubstitution([
            FindPackageShare("inspection_cell_description"), 
            "config", 
            "joint_limits.yaml"
        ]),
        " ",
        "kinematics_params:=",
        PathJoinSubstitution([
            FindPackageShare("inspection_cell_description"), 
            "config", 
            "my_robot_calibration.yaml"
        ]),
        " ",
        "physical_params:=",
        PathJoinSubstitution([
            FindPackageShare("inspection_cell_description"), 
            "config", 
            "physical_parameters.yaml"
        ]),
        " ",
        "visual_params:=",
        PathJoinSubstitution([
            FindPackageShare("inspection_cell_description"), 
            "config", 
            "visual_parameters.yaml"
        ]),
        " ",
        "initial_positions_file:=",
        PathJoinSubstitution([
            FindPackageShare("inspection_cell_description"),
            "config", 
            "initial_positions.yaml"
        ]),
    ])

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    moveit_controllers_path = os.path.join(
        get_package_share_directory("inspection_cell_moveit_config"),
        "config",
        "moveit_controllers.yaml",
    )

    moveit_config = (
        MoveItConfigsBuilder("inspection_cell", package_name="inspection_cell_moveit_config")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path=moveit_controllers_path)
        .planning_pipelines(
            pipelines=["ompl", "chomp"]
        )
        .to_moveit_configs()
    )

    #moveit_config = MoveItConfigsBuilder("inspection_cell", package_name="inspection_cell_moveit_config").trajectory_execution().to_moveit_configs()

    ur_dashboard_client = Node(
        package="ur_robot_driver",
        executable="dashboard_client", 
        name="ur_dashboard_client",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("inspection_cell_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # name="controller_manager",
        parameters=[
            ros2_controllers_path,
            ur5e_update_rate_config,
        ],
        output="screen",
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    inspection_cell_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="inspection_cell_controller_spawner",
        arguments=[
            "inspection_cell_controllerv1",
            "--controller-manager", "/controller_manager", 
        ],
        output="screen",
    )

    # # 8. BACKUP: UR-only controller (inactive)
    ur_controller_spawner = Node(
        package="controller_manager", 
        executable="spawner",
        name="ur_controller_spawner",
        arguments=[
            "ur5e_controllerv1",
            "--controller-manager", "/controller_manager",
            "--inactive"
        ],
        output="screen",
    )

    # # 9. BACKUP: Turntable-only controller (inactive)
    turntable_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner", 
        name="turntable_trajectory_controller_spawner",
        arguments=[
            "turntable_trajectory_controllerv1",
            "--controller-manager", "/controller_manager",
            "--inactive"
        ],
        output="screen",
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("inspection_cell_moveit_config"),
                "launch",
                "moveit_rviz.launch.py"
            ])
        ]),
    )
    
        
    return LaunchDescription(
        [
            robot_state_publisher,
            controller_manager,
            ur_dashboard_client,
            # joint_state_broadcaster_spawner,
            inspection_cell_controller_spawner,
            # turntable_trajectory_controller_spawner,
            # ur_controller_spawner,
            TimerAction(
                period=3.0,
                actions=[move_group_node]
            ),
            rviz_launch
        ]
    )