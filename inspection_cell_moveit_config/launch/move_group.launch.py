from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ur_moveit_config.launch_common import load_yaml
from launch_ros.actions import Node


def generate_launch_description():
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

    all_actions = [move_group_launch, moveit_py_node, servo_node]

    return LaunchDescription(all_actions)
