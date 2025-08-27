from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from ur_moveit_config.launch_common import load_yaml
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import TimerAction, OpaqueFunction


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("cell", default_value="alpha",
                              choices=["alpha", "beta"]),
        DeclareLaunchArgument("sim", default_value="false",),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("mock_sensor_commands", default_value="false",
                              description="Enable fake command interfaces for sensors used for simple simulations. "
                              "Used only if 'use_fake_hardware' parameter is true."),
        DeclareLaunchArgument("headless_mode", default_value="false",
                              description="Run in headless mode (without GUI)."),
        DeclareLaunchArgument("safety_limits", default_value="true",
                              description="Enable safety limits controller."),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15",
                              description="Safety margin for position limits."),
        DeclareLaunchArgument("safety_k_position", default_value="20",
                              description="k-position factor in safety controller."),
        DeclareLaunchArgument("use_tool_communication", default_value="false",
                              description="Enable tool communication for UR robots."),
        DeclareLaunchArgument("launch_rviz", default_value="true",
                              description="Launch RViz for visualization."),
        DeclareLaunchArgument("launch_moveit", default_value="true",
                              description="Launch MoveIt for motion planning."),
    ]

    cell = LaunchConfiguration("cell")
    sim = LaunchConfiguration("sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    headless_mode = LaunchConfiguration("headless_mode")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_moveit = LaunchConfiguration("launch_moveit")
    use_tool_communication = LaunchConfiguration("use_tool_communication")

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("inspection_cell_description"),
                "launch",
                "inspection_cell_control.launch.py"
            ])
        ]),
        launch_arguments={
            "cell": cell,
            "sim": sim,
            "use_fake_hardware": use_fake_hardware,
            "mock_sensor_commands": mock_sensor_commands,
            "headless_mode": headless_mode,
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
            "use_tool_communication": use_tool_communication,
            "launch_rviz": "false",
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("inspection_cell_moveit_config"),
                "launch", "move_group.launch.py"  # Use the move_group + moveit_py node
            ])
        ),
        launch_arguments={
            "cell": LaunchConfiguration("cell"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "mock_sensor_commands": LaunchConfiguration("mock_sensor_commands"),
            "headless_mode": LaunchConfiguration("headless_mode"),

        }.items(),
        condition=IfCondition(launch_moveit)
    )

    return LaunchDescription([
        control_launch,
        moveit_launch,
    ])
