#!/usr/bin/env python3
"""
Inspection Cell Hardware Launch File - FIXED VERSION
Based on test_sim.launch.py structure but for real hardware
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from ur_moveit_config.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):
    """
    Launch setup function with context access for dynamic configuration
    """

    # ================================================================
    # INITIALIZE LAUNCH CONFIGURATIONS WITH CONTEXT
    # ================================================================
    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_moveit = LaunchConfiguration("launch_moveit")
    # controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    use_tool_communication = LaunchConfiguration("use_tool_communication")

    # ================================================================
    # DYNAMIC CONFIGURATION WITH CONTEXT
    # ================================================================

    # Resolve ur_type for dynamic configuration
    ur_type_value = ur_type.perform(context)

    # UR update rate configuration
    ur_update_rate_config = PathJoinSubstitution([
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

    # ================================================================
    # CONFIGURATION FILES
    # ================================================================

    # Controllers configuration for the unified system
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("inspection_cell_description"),
        "config",
        "ros2_controllers.yaml"
    ])

    # ================================================================
    # CORE SYSTEM NODES
    # ================================================================

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # 2. Controller Manager (SEPARATE NODE - Critical Fix!)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # name="controller_manager",
        parameters=[
            # robot_description,
            ur_update_rate_config,
            controllers_yaml,
        ],
        output="screen",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description")],
    )

    # 3. Joint State Aggregator
    joint_state_aggregator = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_aggregator",
        parameters=[
            {
                "source_list": [
                    "/ur_joint_states",          # From UR (remapped)
                    "/turntables/joint_states"   # From turntable Pi
                ],
                "rate": 50.0,
                "use_sim_time": False,
            }
        ],
        output="screen",
    )

    # ================================================================
    # HARDWARE INTERFACES
    # ================================================================
    # NOTE: Turntable controller already running on Raspberry Pi
    # We can see /turntables/joint_states topic via ROS domain ID

    # 4. UR Dashboard Client (for robot status)
    ur_dashboard_client = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="ur_dashboard_client",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
    )

    # ================================================================
    # CONTROLLER SPAWNERS (Simplified and Fast)
    # ================================================================

    # 6. Joint State Broadcaster (with remapping)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        remappings=[
            ("joint_states", "ur_joint_states"),
        ],
        output="screen",
    )

    # # 7. PRIMARY: Unified 7-DOF Inspection Cell Controller
    inspection_cell_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="inspection_cell_controller_spawner",
        arguments=[
            "inspection_cell_controller",
            "--controller-manager", "/controller_manager",
            "--inactive"],
        output="screen",
    )
    # # 8. SECONDARY: Unified 7-DOF Inspection Cell Forward Position Controller (inactive)
    inspection_cell_forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="inspection_cell_forward_position_controller_spawner",
        arguments=[
            "inspection_cell_forward_position_controller",
            "--controller-manager", "/controller_manager",
            "--inactive"
        ],
        output="screen",
    )

    # # 9. BACKUP: UR-only controller (inactive)
    ur_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="ur_controller_spawner",
        arguments=[
            "ur5e_controller",
            "--controller-manager", "/controller_manager",
            # "--inactive"
        ],
        output="screen",
    )

    # # 10. BACKUP: Turntable-only controller (inactive)
    turntable_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="turntable_trajectory_controller_spawner",
        arguments=[
            "turntable_trajectory_controller",
            "--controller-manager", "/controller_manager",
            # "--inactive"
        ],
        output="screen",
    )

    ur5e_forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur5e_forward_position_controller",
                   "--controller-manager", "/controller_manager", "--inactive"],
        output="screen",
    )

    turntable_forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["turntable_forward_position_controller",
                   "--controller-manager", "/controller_manager", "--inactive"],
        output="screen",
    )

    # ================================================================
    # MOTION PLANNING AND VISUALIZATION
    # ================================================================

    # 12. MoveIt Move Group
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("inspection_cell_moveit_config"),
                "launch",
                "move_group.launch.py"
            ])
        ]),
        condition=IfCondition(launch_moveit)
    )

    # 13. RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("inspection_cell_moveit_config"),
                "launch",
                "moveit_rviz.launch.py"
            ])
        ]),
        condition=IfCondition(launch_rviz)
    )

    # ================================================================
    # RETURN ALL NODES AND EVENT HANDLERS
    # ================================================================

    return [
        robot_state_publisher,
        controller_manager,
        ur_dashboard_client,
        joint_state_broadcaster_spawner,
        inspection_cell_controller_spawner,
        inspection_cell_forward_position_controller_spawner,
        ur_controller_spawner,
        ur5e_forward_position_controller_spawner,
        turntable_trajectory_controller_spawner,
        turntable_forward_position_controller_spawner,
        TimerAction(
            period=2.0, actions=[moveit_launch]),
        rviz_launch,
    ]


def generate_launch_description():
    """
    Generate launch description with declared arguments
    """

    # ================================================================
    # DECLARE LAUNCH ARGUMENTS
    # ================================================================
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.102",
            description="IP address of the UR5e robot"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type of UR robot"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enable UR safety limits"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Run without teach pendant GUI"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz for visualization"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_moveit",
            default_value="true",
            description="Launch MoveIt for motion planning"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="30",
            description="Timeout for controller spawning"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Enable UR tool communication"
        )
    )

    # ================================================================
    # RETURN LAUNCH DESCRIPTION
    # ================================================================

    return LaunchDescription(
        declared_arguments + [
            OpaqueFunction(function=launch_setup)
        ]
    )
