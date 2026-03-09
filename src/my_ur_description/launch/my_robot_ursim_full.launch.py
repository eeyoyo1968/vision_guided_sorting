import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='127.0.0.1',
        description='IP address of URSim'
    )
    
    robot_ip = LaunchConfiguration('robot_ip')
    
    # Include main robot launch
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('my_ur_description'),
                'launch',
                'my_robot.launch.py'
            )
        ]),
        launch_arguments={
            'use_fake_hardware': 'false',
            'robot_ip': robot_ip,
        }.items()
    )
    
    # Gripper bridge node
    gripper_bridge = Node(
        package='my_ur_description',
        executable='robotiq_ursim_bridge.py',
        name='robotiq_ursim_bridge',
        parameters=[{'robot_ip': robot_ip}],
        output='screen'
    )
    
    return LaunchDescription([
        robot_ip_arg,
        robot_launch,
        gripper_bridge,
    ])