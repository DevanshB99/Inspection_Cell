import xacro
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ur_moveit_config.launch_common import load_yaml
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": False,
        "publish_robot_description_semantic": True,
    }

    moveit_config = (
        MoveItConfigsBuilder(
            "inspection_cell", package_name="inspection_cell_moveit_config")
        .trajectory_execution(moveit_manage_controllers=True)
        .planning_scene_monitor(planning_scene_monitor_parameters)
        .moveit_cpp(file_path="config/motion_planning_config.yaml")
        .robot_description("config/inspection_cell.urdf.xacro")
        .robot_description_semantic(
            file_path="config/inspection_cell.srdf")
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

    moveit_py_node = Node(
        name="moveit_py",
        package="viewpoint_generation",
        executable="viewpoint_traversal_node",
        output="both",
        parameters=[
            moveit_config.to_dict()],
        prefix="bash -c 'sleep 5; $0 $@' ",  # Longer delay for safety
    )

    all_actions = [moveit_py_node]

    return LaunchDescription(all_actions)
