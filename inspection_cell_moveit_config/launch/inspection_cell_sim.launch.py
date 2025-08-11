from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from ur_moveit_config.launch_common import load_yaml
from launch_ros.substitutions import FindPackageShare


def generate_launch_description(context, *args, **kwargs):
    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument("sim", default_value="false",), 
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("mock_sensor_commands", default_value="false",
                              description="Enable fake command interfaces for sensors used for simple simulations. "
                              "Used only if 'use_fake_hardware' parameter is true."),
        DeclareLaunchArgument("headless_mode", default_value="false",
                              description="Run in headless mode (without GUI)."),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.102",
                              description="IP address of the robot."),
        DeclareLaunchArgument("safety_limits", default_value="true",
                              description="Enable safety limits controller."),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15",
                              description="Safety margin for position limits."),
        DeclareLaunchArgument("safety_k_position", default_value="20",
                              description="k-position factor in safety controller."),
    ]

    ur_type = LaunchConfiguration("ur_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    headless_mode = LaunchConfiguration("headless_mode")
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_moveit = LaunchConfiguration("launch_moveit")
    use_tool_communication = LaunchConfiguration("use_tool_communication")

    # Determining simulation mode v.s physical hardware mode

    use_fake_hardware = use_fake_hardware.perform(context)
    is_sim = use_fake_hardware == "true"

    # Initial_positions_file path
    initial_positions_file_path = PathJoinSubstitution(
        [FindPackageShare("inspection_cell_description"),
         "config", "initial_positions.yaml"]
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
    ]) if not is_sim else None

    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_client_library"), "resources",
         "external_control.urscript"]
    )
    
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources",
         "rtde_input_recipe.txt"]
    )
    
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources",
         "rtde_output_recipe.txt"]
    )

    joint_limits_params = PathJoinSubstitution([
        FindPackageShare("inspection_cell_description"),
        "config", "joint_limits.yaml"
    ])

    kinematics_params = PathJoinSubstitution([
        FindPackageShare("inspection_cell_description"),
        "config", "kinematics.yaml"
    ])

    physical_params = PathJoinSubstitution([
        FindPackageShare("inspection_cell_description"),
        "config", "physical_parameters.yaml"
    ])
    
    visual_params = PathJoinSubstitution([
        FindPackageShare("inspection_cell_description"),
        "config", "visual_parameters.yaml"
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(
            "inspection_cell_description"), "urdf", "inspection_cell.urdf.xacro"]),
        " ",
        "name:=",
        "inspection_cell",
        " ",
        "ur_type:=",
        ur_type,
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
        "robot_ip:=",
        robot_ip,
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
        script_filename,
        " ",
        "input_recipe_filename:=",
        input_recipe_filename,
        " ",
        "output_recipe_filename:=",
        output_recipe_filename,
        " ",
        "initial_positions_file:=",
        initial_positions_file_path,
    ])

    if not is_sim:
        robot_description_content.extend([" ","use_tool_communication:=", use_tool_communication,
                                            " ", "joint_limit_params:=", PathJoinSubstitution([
                                                FindPackageShare("inspection_cell_description"),
                                                "config", "joint_limits.yaml"
                                            ]),
                                            " ",
                                            "kinematics_params:=",
                                            PathJoinSubstitution([
                                                FindPackageShare("inspection_cell_description"),
                                                "config", "my_robot_calibration.yaml"
                                            ]),
                                            " ",
                                            "physical_params:=",
                                            PathJoinSubstitution([
                                                FindPackageShare("inspection_cell_description"),
                                                "config", "physical_parameters.yaml"
                                            ]),
                                            " ",
                                            "visual_params:=",
                                            PathJoinSubstitution([
                                                FindPackageShare("inspection_cell_description"),
                                                "config", "visual_parameters.yaml"
                                            ]),
                                        ])






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

    # ros2_control Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml, robot_description],
        output="screen",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description")],
    )

    # Controller Spawners
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    ur5e_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur5e_controller",
                   "--controller-manager", "/controller_manager", "--inactive"],
        output="screen",
    )

    turntable_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["turntable_trajectory_controller",
                   "--controller-manager", "/controller_manager", "--inactive"],
        output="screen",
    )

    ur5e_to_turntable_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["inspection_cell_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    inspection_cell_forward_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["inspection_cell_forward_position_controller",
                   "--controller-manager", "/controller_manager", "--inactive"],
        output="screen",
    )

    ur5e_forward_position_controller = Node(
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

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("inspection_cell_moveit_config"),
                "launch",
                "move_group.launch.py"  # Use the move_group + moveit_py node
            ])
        )
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare(
            'inspection_cell_description'), "rviz", "view_robot.rviz"
    ])

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            control_node,
            joint_state_broadcaster,
            ur5e_controller,
            turntable_controller,
            ur5e_to_turntable_controller,
            inspection_cell_forward_position_controller,
            ur5e_forward_position_controller,
            turntable_forward_position_controller_spawner,
            moveit_launch,
            rviz_launch,
        ]
    )
