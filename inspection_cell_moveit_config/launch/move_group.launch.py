import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch
from ur_moveit_config.launch_common import load_yaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import TimerAction


def launch_setup(context):
    # Get the actual cell value at launch time
    cell = LaunchConfiguration("cell").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz")

    xacro_mappings = {
        'cell': LaunchConfiguration("cell").perform(context),
        'use_fake_hardware': LaunchConfiguration("use_fake_hardware").perform(context),
        'mock_sensor_commands': LaunchConfiguration("mock_sensor_commands").perform(context),
        'headless_mode': LaunchConfiguration("headless_mode").perform(context),
    }

    joint_limits_file = PathJoinSubstitution([
        FindPackageShare("inspection_cell_description"),
        "config",
        cell,
        "joint_limits.yaml"
    ]).perform(context)

    moveit_config = (
        MoveItConfigsBuilder(
            "inspection_cell", package_name="inspection_cell_moveit_config"
        )
        .robot_description(
            mappings=xacro_mappings)
        .robot_description_semantic(file_path="config/inspection_cell.srdf")
        .moveit_cpp(file_path="config/motion_planning.yaml")
        .joint_limits(file_path=joint_limits_file)
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl"
        )
        .to_moveit_configs()
    )

    move_group_launch = generate_move_group_launch(moveit_config)

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("inspection_cell_moveit_config"),
                "launch", "moveit_rviz.launch.py"
            ])
        ]),
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml(
        "inspection_cell_moveit_config", "config/cell_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ],
        output="screen",
    )

    return [  # move_group_node,
        move_group_launch,
        servo_node,
        rviz_launch,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cell", default_value="beta", choices=[
                                  "alpha", "beta"], description="Inspection cell type"),
            DeclareLaunchArgument("ur_type", default_value="ur5e",
                                  description="Type of UR robot"),
            DeclareLaunchArgument("use_fake_hardware",
                                  default_value="true", description="Sim mode"),
            DeclareLaunchArgument(
                "mock_sensor_commands", default_value="false", description="Mock sensor commands"),
            DeclareLaunchArgument(
                "headless_mode", default_value="false", description="Disable GUI"),
            DeclareLaunchArgument("launch_rviz", default_value="true",
                                  description="Launch RViz for visualization."),
            OpaqueFunction(function=launch_setup)
        ]
    )
