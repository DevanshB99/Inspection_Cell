from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ur_moveit_config.launch_common import load_yaml


def generate_launch_description():

    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
    ]

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    ur_type = LaunchConfiguration("ur_type")

    # ============================================================================
    # 1. ROBOT DESCRIPTION (Essential for MoveItPy)
    # ============================================================================
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(
            "inspection_cell_description"), "urdf", "inspection_cell.urdf.xacro"]),
        " ",
        "name:=inspection_cell",
        " ",
        "ur_type:=", ur_type,
        " ",
        "use_fake_hardware:=", use_fake_hardware,
        " ",
        "mock_sensor_commands:=false",
        " ",
        "headless_mode:=false",
    ])

    robot_description = {"robot_description": ParameterValue(
        robot_description_content, value_type=str)}

    # Robot State Publisher - publishes robot_description and TF transforms
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # ============================================================================
    # 2. CONTROLLERS (Essential for MoveItPy trajectory execution)
    # ============================================================================

    # Controller configuration
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("inspection_cell_description"),
        "config", "ros2_controllers.yaml"
    ])

    # ros2_control Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml, robot_description],
        output="screen",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description")
        ],
    )

    # Joint State Broadcaster (publishes /joint_states - CRITICAL for MoveItPy)
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Main trajectory controller that MoveItPy will use
    inspection_cell_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["inspection_cell_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # ============================================================================
    # 3. MOVEIT CONFIGURATION + YOUR NODE
    # ============================================================================

    # MoveIt configuration - includes SRDF, kinematics, planning configs
    moveit_config = (
        MoveItConfigsBuilder(
            "inspection_cell",
            package_name="inspection_cell_moveit_config")
        .robot_description_semantic(file_path="config/inspection_cell.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .moveit_cpp(file_path="config/motion_planning_config.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .trajectory_execution(moveit_manage_controllers=True)
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    move_group_launch = generate_move_group_launch(moveit_config)

    # Your MoveItPy Viewpoint Traversal Node
    moveit_py_node = Node(
        name="viewpoint_traversal_node",
        package="viewpoint_generation",
        executable="viewpoint_traversal_node",
        output="both",
        parameters=[moveit_config.to_dict()],
        # Wait for controllers to be ready
        prefix="bash -c 'sleep 5; $0 $@' ",
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml(
        "inspection_cell_moveit_config", "config/cell_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        # namespace="inspection_cell",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    # ============================================================================
    # 4. RVIZ for visualization
    # ============================================================================

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ]
    )

    # ============================================================================
    # LAUNCH SEQUENCE
    # ============================================================================

    return LaunchDescription(
        declared_arguments + [
            # 1. Robot description first
            robot_state_publisher_node,

            # 2. Controllers
            control_node,
            joint_state_broadcaster,
            inspection_cell_controller,

            # 4. Move Group launch
            move_group_launch,

            # 3. MoveItPy node (with delay)
            moveit_py_node,

            # 5. Servo node
            servo_node,

            # 5. Visualization
            rviz_node,
        ]
    )
