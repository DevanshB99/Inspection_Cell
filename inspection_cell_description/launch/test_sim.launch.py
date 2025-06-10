#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # -------------------------------
    # Declare launch arguments
    # -------------------------------
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e", description="Type of UR robot"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true", description="Sim mode"),
        DeclareLaunchArgument("mock_sensor_commands", default_value="false", description="Mock sensor commands"),
        DeclareLaunchArgument("headless_mode", default_value="false", description="Disable GUI"),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz2")
    ]

    # -------------------------------
    # Launch Configs
    # -------------------------------
    ur_type = LaunchConfiguration("ur_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # -------------------------------
    # Package paths
    # -------------------------------
    pkg_descr = FindPackageShare("inspection_cell_description")
    pkg_ur_descr = FindPackageShare("ur_description")
    pkg_ur_driver = FindPackageShare("ur_robot_driver")

    # -------------------------------
    # Xacro - robot_description
    # -------------------------------
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_descr, "urdf", "inspection_cell.urdf.xacro"]),
            " ",
            "name:=inspection_cell",
            " ",
            "ur_type:=", ur_type,
            " ",
            "use_fake_hardware:=", use_fake_hardware,
            " ",
            "mock_sensor_commands:=", mock_sensor_commands,
            " ",
            "headless_mode:=", headless_mode,
            " ",
            "robot_ip:=0.0.0.0",
            " ",
            "safety_limits:=true",
            " ",
            "safety_pos_margin:=0.15",
            " ",
            "safety_k_position:=20",
            " ",
            "joint_limit_params:=", PathJoinSubstitution([pkg_descr, "config", "joint_limits.yaml"]),
            " ",
            "kinematics_params:=", PathJoinSubstitution([pkg_descr, "config", "my_robot_calibration.yaml"]),
            " ",
            "physical_params:=", PathJoinSubstitution([pkg_descr, "config", "physical_parameters.yaml"]),
            " ",
            "visual_params:=", PathJoinSubstitution([pkg_descr, "config", "visual_parameters.yaml"]),
            " ",
            "script_filename:=", PathJoinSubstitution([pkg_ur_driver, "resources", "external_control.urscript"]),
            " ",
            "input_recipe_filename:=", PathJoinSubstitution([pkg_ur_driver, "resources", "rtde_input_recipe.txt"]),
            " ",
            "output_recipe_filename:=", PathJoinSubstitution([pkg_ur_driver, "resources", "rtde_output_recipe.txt"]),
            " ",
            "initial_positions_file:=", PathJoinSubstitution([pkg_descr, "config", "initial_positions.yaml"]),
            " ",
            "hash:=calib_unified_7dof_12345678901234567890"
        ]),
        value_type=str
    )

    robot_description = {"robot_description": robot_description_content}

    # -------------------------------
    # Controller YAML config
    # -------------------------------
    robot_controllers = PathJoinSubstitution([
        pkg_descr, "config", "ros2_controllers.yaml"
    ])

    # -------------------------------
    # Nodes
    # -------------------------------
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="screen",
        remappings=[("/controller_manager/robot_description", "/robot_description")]
    )

    joint_state_pub_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(headless_mode),
        name="joint_state_publisher_gui",
        output="screen"
    )

    # Spawner nodes (deferred to allow controller_manager to start)
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["inspection_cell_controller"],
            output="screen"
        ),
        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["ur5e_controller"],
        #     output="screen"
        # ),
        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["turntable_trajectory_controller"],
        #     output="screen"
        # ),
        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["forward_position_controller"],
        #     output="screen"
        # ),
        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["forward_velocity_controller"],
        #     output="screen"
        # )
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(launch_rviz),
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([pkg_descr, "rviz", "inspection_cell.rviz"])]
    )

    return LaunchDescription(
        declared_arguments +
        [rsp_node, control_node, joint_state_pub_gui] +
        spawners +
        [rviz_node]
    )
