from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Include the main UR Driver launch
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": "ur12e",
            "robot_ip": "172.17.0.2",
            "description_package": "ur_workcell",
            "description_file": "ur12e_workcell.urdf.xacro",
            "launch_rviz": "true",
        }.items(),
    )

    # 2. Gripper Spawner Node
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # 3. Delay the gripper spawner slightly to ensure controller_manager is up
    delayed_gripper_spawner = TimerAction(
        period=5.0,
        actions=[gripper_spawner]
    )

    return LaunchDescription([
        ur_driver_launch,
        delayed_gripper_spawner
    ])