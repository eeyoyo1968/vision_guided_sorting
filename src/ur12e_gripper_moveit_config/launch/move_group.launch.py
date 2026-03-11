import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Construct the MoveIt configuration
    # Note: Ensure these file names match your actual files in the config folder
    moveit_config = (
        MoveItConfigsBuilder("ur12e_workcell", package_name="ur12e_gripper_moveit_config")
        .robot_description(file_path="config/ur12e_workcell.urdf.xacro")
        .robot_description_semantic(file_path="config/ur12e_workcell.srdf")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # MoveGroup node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # Static TF (Optional, but helps if markers don't appear)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0", "0", "0", "world", "base_link"],
    )

    return LaunchDescription([
        static_tf,
        run_move_group_node,
    ])