from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ur_moveit_config.launch_common import load_yaml
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "inspection_cell", package_name="inspection_cell_moveit_config")
        .trajectory_execution(moveit_manage_controllers=True)
        .to_moveit_configs()
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

    move_group_launch = generate_move_group_launch(moveit_config)
    all_actions = move_group_launch.entities + [servo_node]

    return LaunchDescription(all_actions)
