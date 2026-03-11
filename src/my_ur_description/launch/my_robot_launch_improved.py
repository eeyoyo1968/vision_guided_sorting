import os
import yaml
import math
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    
    def degrees_constructor(loader, node):
        value = loader.construct_scalar(node)
        return math.radians(float(value))

    yaml.SafeLoader.add_constructor('!degrees', degrees_constructor)

    try:
        with open(absolute_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception: 
        return None

def launch_setup(context, *args, **kwargs):
    """
    This function is called after launch arguments are resolved,
    allowing us to make decisions based on their actual values.
    """
    description_pkg = "my_ur_description"
    
    # Get the actual value of use_fake_hardware (as string)
    use_fake_hardware_str = LaunchConfiguration("use_fake_hardware").perform(context)
    use_fake_hardware_bool = use_fake_hardware_str.lower() == "true"
    
    robot_ip_str = LaunchConfiguration("robot_ip").perform(context)
    
    # Robot Description (XACRO)
    xacro_file = PathJoinSubstitution([
        get_package_share_directory(description_pkg), 
        'urdf', 
        'ur_system.xacro'
    ])
    
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", xacro_file,
        " ", "use_fake_hardware:=", use_fake_hardware_str,
        " ", "robot_ip:=", robot_ip_str,
    ])
    robot_description = {'robot_description': robot_description_content}

    # Load SRDF - process xacro to get final SRDF
    srdf_xacro_file = PathJoinSubstitution([
        get_package_share_directory(description_pkg), 
        'urdf', 
        'ur_srdf.xacro'
    ])
    
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", srdf_xacro_file,
    ])
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    # Load configs
    kinematics_yaml = load_yaml(description_pkg, 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml(description_pkg, "config/joint_limits.yaml")
    controllers_yaml = os.path.join(
        get_package_share_directory(description_pkg), 
        'config', 
        'ur_controllers.yaml'
    )
    rviz_config_file = os.path.join(
        get_package_share_directory(description_pkg), 
        'rviz', 
        'my_robot_view.rviz'
    )

    # Select the appropriate arm controller based on hardware type
    if use_fake_hardware_bool:
        arm_controller_name = "joint_trajectory_controller"
    else:
        arm_controller_name = "scaled_joint_trajectory_controller"

    # MoveIt Controller Configuration
    moveit_controllers = {
        'moveit_simple_controller_manager': {
            'controller_names': [arm_controller_name, 'robotiq_gripper_controller'],
            
            arm_controller_name: {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': [
                    'shoulder_pan_joint', 
                    'shoulder_lift_joint', 
                    'elbow_joint', 
                    'wrist_1_joint', 
                    'wrist_2_joint', 
                    'wrist_3_joint'
                ],
            },
            'robotiq_gripper_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'joints': ['robotiq_85_left_knuckle_joint'],
            },
        }
    }

    # Planning pipeline configuration
    planning_pipeline_config = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization ' \
                                'default_planner_request_adapters/FixWorkspaceBounds ' \
                                'default_planner_request_adapters/FixStartStateBounds ' \
                                'default_planner_request_adapters/FixStartStateCollision ' \
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
            'longest_valid_segment_fraction': 0.005,
        }
    }

    # Build the nodes list
    nodes_to_launch = [
        # Robot State Publisher
        Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            output='both', 
            parameters=[robot_description, {'use_sim_time': False}]
        ),
        
        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                robot_description, 
                {'use_sim_time': False, 'update_rate': 100}, 
                controllers_yaml
            ],
            output='both',
        ),

        # MoveIt Move Group
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                {'robot_description_kinematics': kinematics_yaml},
                planning_pipeline_config,
                moveit_controllers,
                {
                    'use_sim_time': False, 
                    'publish_planning_scene': True, 
                    'robot_description_planning': joint_limits_yaml
                }
            ],
        ),

        # Spawn joint_state_broadcaster (always needed)
        Node(
            package='controller_manager', 
            executable='spawner', 
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        # Spawn arm controller - conditional on hardware type
        Node(
            package='controller_manager', 
            executable='spawner', 
            arguments=['joint_trajectory_controller'],
            condition=IfCondition(use_fake_hardware_str),
            output='screen',
        ),

        Node(
            package='controller_manager', 
            executable='spawner', 
            arguments=['scaled_joint_trajectory_controller'],
            condition=UnlessCondition(use_fake_hardware_str),
            output='screen',
        ),

        # Spawn gripper controller (always needed)
        #Node(
        #    package='controller_manager', 
        #    executable='spawner', 
        #    arguments=['robotiq_gripper_controller'],
        #    output='screen',
        #),

        # Launch RViz after a delay
        TimerAction(
            period=5.0, 
            actions=[
                Node(
                    package='rviz2', 
                    executable='rviz2', 
                    name='rviz2', 
                    output='screen',
                    arguments=['-d', rviz_config_file],
                    parameters=[
                        robot_description, 
                        robot_description_semantic, 
                        {'robot_description_kinematics': kinematics_yaml}, 
                        planning_pipeline_config
                    ],
                )
            ]
        )
    ]

    return nodes_to_launch


def generate_launch_description():
    """
    Main launch description generator.
    Declares arguments and uses OpaqueFunction to defer node creation
    until arguments are resolved.
    """
    
    # Declare launch arguments
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware", 
        default_value="true",
        description="Use fake hardware (mock_components) or real UR driver"
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip", 
        default_value="127.0.0.1",
        description="IP address of the UR robot (for URSim, use 127.0.0.1)"
    )

    return LaunchDescription([
        use_fake_hardware_arg,
        robot_ip_arg,
        # Use OpaqueFunction to defer node creation until args are resolved
        OpaqueFunction(function=launch_setup)
    ])
