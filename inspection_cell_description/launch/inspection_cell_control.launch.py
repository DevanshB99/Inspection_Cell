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


def launch_setup(context):

    cell = LaunchConfiguration("cell")
    sim = LaunchConfiguration("sim")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    headless_mode = LaunchConfiguration("headless_mode")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    controller_spawner_timeout = LaunchConfiguration(
        "controller_spawner_timeout")
    active_joint_controller = LaunchConfiguration(
        "active_joint_controller")
    initial_joint_controller = LaunchConfiguration(
        "initial_joint_controller")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_tool_communication = LaunchConfiguration("use_tool_communication")

    # Initial_positions_file path
    initial_positions_file_path = PathJoinSubstitution(
        [FindPackageShare("inspection_cell_description"),
         "config", cell, "initial_positions.yaml"]
    )

    # Controller Config
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare(
            "inspection_cell_description"), "config", "ros2_controllers.yaml"
    ])

    # UR Script and recipe files paths
    ur_update_rate_config = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),
        "config", "ur5e_update_rate.yaml"
    ])

    # Robot description command
    robot_description_command = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("inspection_cell_description"),
            "urdf",
            "inspection_cell.urdf.xacro",
        ]),
        " ",
        "name:=",
        "inspection_cell",
        " ",
        "cell:=",
        cell,
        " ",
        "use_fake_hardware:=",
        use_fake_hardware,
        " ",
        "mock_sensor_commands:=",
        mock_sensor_commands,
        " ",
        "headless_mode:=",
        headless_mode,
        " ",
        "safety_limits:=",
        safety_limits,
        " ",
        "safety_pos_margin:=",
        safety_pos_margin,
        " ",
        "safety_k_position:=",
        safety_k_position,
        " ",
        "script_filename:=",
        PathJoinSubstitution(
            [FindPackageShare("ur_client_library"), "resources",
             "external_control.urscript"]
        ),
        " ",
        "input_recipe_filename:=",
        PathJoinSubstitution(
            [FindPackageShare("ur_robot_driver"), "resources",
             "rtde_input_recipe.txt"]
        ),
        " ",
        "output_recipe_filename:=",
        PathJoinSubstitution(
            [FindPackageShare("ur_robot_driver"), "resources",
             "rtde_output_recipe.txt"]
        ),
        " ",
        "initial_positions_file:=",
        initial_positions_file_path,
        " ",
        "use_tool_communication:=",
        use_tool_communication,
        " ",
        "joint_limit_params:=",
        PathJoinSubstitution([
            FindPackageShare("inspection_cell_description"),
            "config", cell, "joint_limits.yaml"]),
        " ",
        "kinematics_params:=",
        PathJoinSubstitution([
            FindPackageShare("inspection_cell_description"),
            "config", cell, "my_robot_calibration.yaml"]),
        " ",
        "physical_params:=",
        PathJoinSubstitution([
            FindPackageShare("inspection_cell_description"),
            "config", cell, "physical_parameters.yaml"]),
        " ",
        "visual_params:=",
        PathJoinSubstitution([
            FindPackageShare("inspection_cell_description"),
            "config", cell, "visual_parameters.yaml"]),
    ]

    robot_description_content = Command(robot_description_command)
    robot_description = {"robot_description": ParameterValue(
        robot_description_content, value_type=str)}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
        arguments=["--log-level", "debug"],
    )

    controller_params = [controllers_yaml, robot_description]
    if not sim:
        # Load UR update rate configuration if not in simulation mode
        controller_params.insert(0, ur_update_rate_config)

    # ros2_control Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=controller_params,
        output="screen",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description")],
    )

    # ================================================================
    # HARDWARE-SPECIFIC NODE
    # ================================================================
    if cell.perform(context) == "alpha":
        robot_ip = "192.168.0.12"
    elif cell.perform(context) == "beta":
        robot_ip = "192.168.0.112"

    dashboard_client_node = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="ur_dashboard_client",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
        condition=UnlessCondition(sim)
    )

    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"robot_ip": robot_ip},
        ],
    )

    # tool_communication_node = Node(
    #     package="ur_robot_driver",
    #     condition=IfCondition(use_tool_communication),
    #     executable="tool_communication.py",
    #     name="ur_tool_comm",
    #     output="screen",
    #     parameters=[
    #         {
    #             "robot_ip": robot_ip,
    #             "tcp_port": tool_tcp_port,
    #             "device_name": tool_device_name,
    #         }
    #     ],
    # )

    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": "true"},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                    "tcp_pose_broadcaster",
                    "ur_configuration_controller",
                ]
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(launch_rviz),
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("inspection_cell_description"),
            "config",
            "inspection_cell.rviz"
        ])]
    )

    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        "joint_state_broadcaster",
        # "io_and_status_controller",
        # "speed_scaling_state_broadcaster",
        # "force_torque_sensor_broadcaster",
        # "tcp_pose_broadcaster",
        # "ur_configuration_controller",
        "ur5e_forward_position_controller",
        "turntable_forward_position_controller",
    ]
    controllers_inactive = [
        "inspection_cell_controller",
        "inspection_cell_forward_position_controller",
        "ur5e_controller",
        "turntable_trajectory_controller",
        # "force_mode_controller",
        # "passthrough_trajectory_controller",
        # "freedrive_mode_controller",
        # "tool_contact_controller",
    ]
    if active_joint_controller.perform(context) == "false":
        controllers_inactive.append(
            initial_joint_controller.perform(context))
        controllers_active.remove(
            initial_joint_controller.perform(context))

    if use_fake_hardware.perform(context) == "true":
        controllers_active.remove("tcp_pose_broadcaster")

    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]

    nodes_to_start = [
        control_node,
        dashboard_client_node,
        robot_state_helper_node,
        # tool_communication_node,
        controller_stopper_node,
        urscript_interface,
        robot_state_publisher_node,
        rviz_node
    ] + controller_spawners

    return nodes_to_start


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
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        ),
        DeclareLaunchArgument(
            "active_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="ur5e_forward_position_controller",
            description="Activate loaded joint controller.",
        ),
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
