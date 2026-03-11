import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint,PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
# Ensure Point, Quaternion, AND Pose are imported here
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose 
from action_msgs.msg import GoalStatus
import time
import numpy as np
import math
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
import threading
from controller_manager_msgs.srv import SwitchController
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener




# Add the missing service import from our last step
from moveit_msgs.srv import GetPositionIK, GetCartesianPath

class UR12eExtendedPlanner(Node):
    def __init__(self):
        super().__init__('ur12e_extended_planner')
        
        # Action Clients
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._direct_arm_client = ActionClient(
            self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # IK Service Client
        self._ik_client = self.create_client(GetPositionIK, 'compute_ik')
        
        # Cartesian path  NOT working, robot may go crazy!!!!
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')

        # for urscript
        self.script_pub = self.create_publisher(String, '/urscript_interface', 10)

        # for wrench control
        self.current_force_z = 0.0
        self.force_monitor_active = False
        self.force_limit_reached = False
        self.active_goal_handle = None


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # Use your specific topic from the list
        self.wrench_sub = self.create_subscription(
            WrenchStamped, 
            '/UR20255101361/tcp_wrench', 
            self.wrench_cb, 
            10
        )


        self.tcp_subscriber = self.create_subscription(
            PoseStamped, 
            '/tcp_pose_broadcaster/pose', 
            self.tcp_cb, 
            10)
        self.current_tcp_pose = None



        # State tracking
        self.current_joints = None
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        # Reference Home for IK Seed
        self.home_seed = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]
        
        self.get_logger().info("UR12e Extended Planner Initialized")

    def tcp_cb(self, msg):
        self.current_tcp_pose = msg.pose
    
    def get_current_pose(self):
        # Give it a moment to receive a message if just starting
        return self.current_tcp_pose


    def wrench_cb(self, msg):
        # Monitor force magnitude on Z axis
        self.current_force_z = abs(msg.wrench.force.z)
    
        # If we are actively looking for contact and hit the limit
        if self.force_monitor_active and self.current_force_z > 1.0: # 7.0 Newtons threshold
            self.get_logger().info(f"Contact detected! Force: {self.current_force_z:.1f}N")
            self.force_limit_reached = True
            self.force_monitor_active = False # Disable to prevent double triggers
            self.cancel_active_move()

    def cancel_active_move(self):
        """Aborts the trajectory and commands an immediate stop."""
        if self.active_goal_handle is not None:
            # 1. Cancel the long-running goal
            self.active_goal_handle.cancel_goal_async()
        
            # 2. EMERGENCY STOP: Send a zero-displacement trajectory 
            # to freeze the joints at their current positions.
            stop_msg = FollowJointTrajectory.Goal()
            stop_msg.trajectory.joint_names = [
                'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
            ]
            # Command current position with a very short time
            p = JointTrajectoryPoint(positions=self.current_joints, 
                                     time_from_start=rclpy.duration.Duration(seconds=0, nanoseconds=50000000).to_msg())
            stop_msg.trajectory.points = [p]
            self._direct_arm_client.send_goal_async(stop_msg)
        
            self.get_logger().warn("Force Stop Triggered.")

    def smart_descent_skill_xyz_theta(self, x,y,z, theta_rad , duration=5.0):
        """
        Commands a slow joint move and monitors force.
        Returns True if contact was hit, False if it reached the target naturally.
        """
        self.force_limit_reached = False
        self.force_monitor_active = True
    
        # Start the async move
        # Using a long duration ensures a slow, safe speed
        future = self.move_xyz_theta_async(x,y,z,theta_rad, duration=duration)
    
        if future is None:
            self.force_monitor_active = False
            return False

        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, future)
        self.active_goal_handle = future.result()
    
        if not self.active_goal_handle.accepted:
            self.force_monitor_active = False
            return False

        # Wait for result OR force trigger
        res_future = self.active_goal_handle.get_result_async()
    
        while rclpy.ok() and not self.force_limit_reached:
            # Check if the move finished naturally
            if res_future.done():
                self.force_monitor_active = False
                return False 
            rclpy.spin_once(self, timeout_sec=0.01)

        self.force_monitor_active = False
        return True # We hit the table

    def joint_cb(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.current_joints = [
                name_to_pos['shoulder_pan_joint'], name_to_pos['shoulder_lift_joint'],
                name_to_pos['elbow_joint'], name_to_pos['wrist_1_joint'],
                name_to_pos['wrist_2_joint'], name_to_pos['wrist_3_joint']
            ]
        except KeyError:
            pass

    # --- CORE JOINT MOVE FUNCTIONS ---

    def jmove_async(self, joint_vector, duration=5.0):
        """Directly commands controller. Returns future immediately."""
        if self.current_joints is None or not self._direct_arm_client.wait_for_server(timeout_sec=2.0):
            return None

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        p1 = JointTrajectoryPoint(positions=self.current_joints, time_from_start=rclpy.duration.Duration(seconds=0).to_msg())
        p2 = JointTrajectoryPoint(positions=[float(j) for j in joint_vector], 
                                  time_from_start=rclpy.duration.Duration(seconds=int(duration)).to_msg())

        goal_msg.trajectory.points = [p1, p2]
        return self._direct_arm_client.send_goal_async(goal_msg)

    def jmove_sync(self, joint_vector, duration=5.0):
        """Blocks until direct controller motion is complete."""
        future = self.jmove_async(joint_vector, duration)
        if future is None: return False
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().status == GoalStatus.STATUS_SUCCEEDED


    def jmove_plan_async(self, joint_vector, speed=0.1):
        """Planned move with collision avoidance (Async)."""
        if not self._move_group_client.wait_for_server(timeout_sec=5.0): return None
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        con = Constraints()
        for name, pos in zip(joint_names, joint_vector):
            jc = JointConstraint(joint_name=name, position=float(pos), 
                                 tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            con.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(con)
        goal_msg.request.max_velocity_scaling_factor = float(speed)
        goal_msg.request.max_acceleration_scaling_factor = float(speed) # Good to scale acceleration too
        return self._move_group_client.send_goal_async(goal_msg)

    def jmove_plan_sync(self, joint_vector, speed=0.1):
        """Planned move with collision avoidance (Sync)."""
        future = self.jmove_plan_async(joint_vector, speed)
        if not future: return False
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle or not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().status == GoalStatus.STATUS_SUCCEEDED

    # --- IK RESOLVER ---

    def get_ik_solution(self, x, y, z, qx, qy, qz, qw, frame_id="base_link"):
        """Solves IK using a reference seed to prevent flipping."""
        if not self._ik_client.wait_for_service(timeout_sec=2.0): return None
        
        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = "ur_manipulator"
        ik_req.avoid_collisions = True
        
        target = PoseStamped()
        target.header.frame_id = frame_id
        target.pose.position = Point(x=float(x), y=float(y), z=float(z))
        target.pose.orientation = Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))
        
        ik_req.pose_stamped = target
        ik_req.robot_state.joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                                              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        ik_req.robot_state.joint_state.position = self.home_seed
        
        req.ik_request = ik_req
        future = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if res.error_code.val == 1:
            # Clean wrist rotation to prevent >180 spins
            joints = list(res.solution.joint_state.position[:6])
            joints[5] = (joints[5] + math.pi) % (2 * math.pi) - math.pi
            return joints
        return None

    # --- CARTESIAN PLANNED MOVES (YOUR REQUEST) ---

    def move_xyz_theta_async(self, x, y, z, theta_rad, duration=5.0, frame_id="base_link"):
        """Asynchronous planned move to XYZ + Rotation (Downwards + Twist)."""
        qx = math.cos(theta_rad / 2.0)
        qy = -math.sin(theta_rad / 2.0)
        qz, qw = 0.0, 0.0 # Standard down-orientation components
        
        joint_sol = self.get_ik_solution(x, y, z, qx, qy, qz, qw, frame_id)
        if joint_sol:
            return self.jmove_async(joint_sol, duration)
        self.get_logger().error("IK failed for move_plan_xyz_theta_async")
        return None
    
    def move_plan_xyz_theta_async(self, x, y, z, theta_rad, speed=0.1, frame_id="base_link"):
        """Asynchronous planned move to XYZ + Rotation (Downwards + Twist)."""
        qx = math.cos(theta_rad / 2.0)
        qy = -math.sin(theta_rad / 2.0)
        qz, qw = 0.0, 0.0 # Standard down-orientation components
        # This creates a quaternion pointing the Z-axis of the tool straight down 
        # while rotating around that axis by theta_rad.
        #qx = math.sin(math.pi/4) * math.cos(theta_rad/2) # Simple version for top-down
        #qy = math.sin(math.pi/4) * math.sin(theta_rad/2)
        #qz = math.cos(math.pi/4) * math.sin(theta_rad/2)
        #qw = math.cos(math.pi/4) * math.cos(theta_rad/2)
        
        joint_sol = self.get_ik_solution(x, y, z, qx, qy, qz, qw, frame_id)
        if joint_sol:
            return self.jmove_plan_async(joint_sol, speed)
        self.get_logger().error("IK failed for move_plan_xyz_theta_async")
        return None

    def move_plan_xyz_theta_sync(self, x, y, z, theta_rad, speed=0.1, frame_id="base_link"):
        """Synchronous planned move to XYZ + Rotation."""
        future = self.move_plan_xyz_theta_async(x, y, z, theta_rad, speed, frame_id)
        if not future: return False
        # The logic is identical to jmove_plan_sync but starts from the async call above
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def move_plan_xyz_async(self, x, y, z, speed=0.1, frame_id="base_link"):
        """Asynchronous planned move to XYZ (Pointing Straight Down)."""
        return self.move_plan_xyz_theta_async(x, y, z, 0.0, speed, frame_id)

    def move_plan_xyz_sync(self, x, y, z, speed=0.1, frame_id="base_link"):
        """Synchronous planned move to XYZ (Pointing Straight Down)."""
        return self.move_plan_xyz_theta_sync(x, y, z, 0.0, speed, frame_id)
    
    def move_pose_plan_async(self, pose, speed=0.1):
        """Planned move to a geometry_msgs/Pose with collision avoidance (Async)."""
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            return None
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
    
        # Use the helper to convert the Pose into Joint Constraints via IK
        # This is more stable for UR arms than raw Pose constraints
        goal_msg.request.goal_constraints.append(self.pose_to_constraints(pose))
    
        goal_msg.request.max_velocity_scaling_factor = float(speed)
        goal_msg.request.max_acceleration_scaling_factor = float(speed)
    
        return self._move_group_client.send_goal_async(goal_msg)

    def move_pose_plan_sync(self, pose, speed=0.1):
        """Planned move to a geometry_msgs/Pose (Sync)."""
        future = self.move_pose_plan_async(pose, speed)
        if not future: return False
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle or not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def pose_to_constraints(self, pose, tolerance=0.01):
        """Helper to convert a Pose into MoveIt Constraints using IK."""
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        joint_sol = self.get_ik_solution(pose.position.x, pose.position.y, pose.position.z, qx, qy, qz, qw)
    
        con = Constraints()
        if joint_sol:
            joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            for name, pos in zip(joint_names, joint_sol):
                # JointConstraint is now available from the top-level import
                jc = JointConstraint(joint_name=name, position=float(pos), 
                                     tolerance_above=tolerance, tolerance_below=tolerance, weight=1.0)
                con.joint_constraints.append(jc)
        else:
            self.get_logger().error("Failed to calculate IK for Pose constraints!")
        return con

    def scale_trajectory_speed(self, trajectory, speed_factor):
        """Scales the timing of a joint trajectory to change execution speed."""
        if speed_factor <= 0 or speed_factor >= 1.0:
            return trajectory # Safety fallback

        new_traj = trajectory
        for i in range(len(new_traj.joint_trajectory.points)):
            point = new_traj.joint_trajectory.points[i]
        
            # FIX: Convert msg Duration to total nanoseconds manually
            current_total_nano = (point.time_from_start.sec * 10**9) + point.time_from_start.nanosec
        
            # Scale the total time
            new_total_nano = int(current_total_nano / speed_factor)
        
            # FIX: Update the message fields directly
            point.time_from_start.sec = int(new_total_nano // 10**9)
            point.time_from_start.nanosec = int(new_total_nano % 10**9)
        
            # Scale velocities and accelerations (smaller speed factor = slower)
            point.velocities = [v * speed_factor for v in point.velocities]
            point.accelerations = [a * speed_factor for a in point.accelerations]
        
        return new_traj

    def execute_cartesian_trajectory(self, trajectory):
        """Helper to send a computed Cartesian trajectory to the robot controller."""
        if not self._direct_arm_client.wait_for_server(timeout_sec=5.0):
            return False
            
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory.joint_trajectory
        
        future = self._direct_arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            return False
            
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def plan_cartesian_path(self, waypoints):
        """Calls MoveIt service with safety checks using provided Pose waypoints."""
        if not self._cartesian_client.wait_for_service(timeout_sec=2.0):
            return None

        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = "ur_manipulator"
        req.waypoints = waypoints
        req.max_step = 0.01 
        req.jump_threshold = 0.0 
        req.avoid_collisions = True

        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        # Only proceed if 100% of the path is possible
        if res and res.fraction >= 1.0:
            return res.solution
        else:
            percent = (res.fraction * 100) if res else 0
            self.get_logger().error(f"Cartesian Path Failed! Only {percent:.1f}% possible.")
            return None
            
    def move_pose_cartesian_path(self, pose, speed=0.1):
        """Executes a straight-line move to a specific geometry_msgs/Pose with speed control."""
        # Plan the path using the existing plan_cartesian_path method
        plan = self.plan_cartesian_path([pose])
    
        if plan:
            self.get_logger().info(f"Executing Pose Cartesian path at speed {speed}...")
            # Scale the trajectory timing before execution
            scaled_plan = self.scale_trajectory_speed(plan, speed)
            return self.execute_cartesian_trajectory(scaled_plan)
        else:
            # Fallback for safety if the straight line is blocked or impossible
            self.get_logger().warn("Pose Cartesian path failed. Falling back to standard planning.")
            # Extract Euler angles if your move_plan_xyz_theta_sync requires them
            return self.move_pose_plan_sync(pose, speed=speed)
        
    def move_xyz_cartesian_path(self, x, y, z, speed=0.1):
        """Straight line move to XYZ (pointing down) with speed control."""
        target = Pose()
        target.position = Point(x=float(x), y=float(y), z=float(z))
        # Standard downward orientation
        target.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
    
        plan = self.plan_cartesian_path([target])
        if plan:
            self.get_logger().info(f"Executing linear XYZ path at speed {speed}...")
            scaled_plan = self.scale_trajectory_speed(plan, speed)
            return self.execute_cartesian_trajectory(scaled_plan)
        else:
            self.get_logger().warn("Cartesian path failed. Falling back to standard planned move.")
            return self.move_plan_xyz_sync(x, y, z, speed=speed)   
             
    def move_xyz_theta_cartesian_path(self, x, y, z, theta_rad, speed=0.1):
        """Straight line move with custom wrist rotation and speed control."""
        # Orientation logic for top-down pick with twist
        qx = math.cos(theta_rad / 2.0)
        qy = -math.sin(theta_rad / 2.0)
        qz, qw = 0.0, 0.0 

        target = Pose()
        target.position = Point(x=float(x), y=float(y), z=float(z))
        target.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    
        plan = self.plan_cartesian_path([target])
    
        if plan:
            self.get_logger().info(f"Executing Cartesian path at speed {speed}...")
            # Scale the plan before sending it to the controller
            scaled_plan = self.scale_trajectory_speed(plan, speed)
            return self.execute_cartesian_trajectory(scaled_plan)
        else:
            self.get_logger().warn("Cartesian path failed. Falling back to curved plan.")
            # Fallback to your standard planned move if straight line is impossible
            return self.move_plan_xyz_theta_sync(x, y, z, theta_rad, speed=speed)   

    def move_until_contact_ros_native(self):
        msg = String()
        msg.data = (
        "def smart_touch():\n"
        "  set_tcp(p[0,0,0,0,0,0])\n"
        "  zero_ftsensor()\n"
        "  sleep(0.1)\n"
        "  # direction, speed, acceleration\n"
        "  # We use a slightly higher force threshold by adding it to the command\n"
        "  stop_on_contact(direction=[0,0,-1], speed=0.03)\n"
        "  \n"
        "  # Explicit stop and retract\n"
        "  stopl(1.2)\n"
        "  p_now = get_actual_tcp_pose()\n"
        "  p_up = pose_add(p_now, p[0,0,0.01,0,0,0])\n"
        "  movel(p_up, a=1.2, v=0.05)\n"
        "end\n"
        "smart_touch()\n"
        )
        self.script_pub.publish(msg)

    def async_move_until_contact(self, x, y, floor_z, angle):
        self.is_monitoring_force = True
        self.stop_requested = False
    
        # 1. Start a SLOW move to the floor (e.g. 1cm/sec)
        # Ensure your move_plan_xyz_theta_sync has an async version 
        # or use the underlying action client directly here.
        self.get_logger().info("Commencing slow async descent...")
    
        # We call the move WITHOUT waiting for it to finish (async)
        self.move_xyz_theta_async(x, y, floor_z, angle)

        # 2. Wait and Monitor
        while rclpy.ok():
            if self.stop_requested:
                break
            # Check if the robot actually reached the floor without hitting anything
            if self.check_move_completed():
                break
            time.sleep(0.01)

        self.is_monitoring_force = False
    
        # 3. Retract 1cm immediately to clear table for gripper
        self.get_logger().info("Contact made or floor reached. Retracting...")
        self.move_plan_xyz_theta_sync(x, y, self.get_current_z() + 0.01, angle)

    def retract_relative_python(self, distance_m=0.004, speed=0.05):
        """Retracts relative to current position using Python/MoveIt logic."""
        # 1. Get the current pose from your TCP subscriber or TF
        # If you have a subscriber to /tcp_pose_broadcaster/pose:
        current_pose = self.get_current_pose() # Assuming you have this helper
    
        if current_pose is None:
            self.get_logger().error("Could not determine current pose for retract!")
            return False

        # 2. Calculate the new target pose
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x
        target_pose.position.y = current_pose.position.y
        target_pose.position.z = current_pose.position.z + distance_m
    
        # Maintain the same orientation
        target_pose.orientation = current_pose.orientation

        self.get_logger().info(f"Retracting to Z: {target_pose.position.z}")

        # 3. Use your existing Cartesian or Sync move
        # Using Cartesian is better for a perfectly straight vertical lift
        return self.move_pose_cartesian_path(target_pose, speed=speed)

    def retract_after_contact(self, distance_m=0.004):
        """Captures the current position ONCE and lifts the robot to a fixed target."""
        # 1. Wait for a fresh pose message to ensure we have the 'stopped' position
        self.current_tcp_pose = None
    
        # We wait up to 1 second for the subscriber to update
        timeout = 1.0
        start_time = time.time()
        while self.current_tcp_pose is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_tcp_pose is None:
            self.get_logger().error("Retract failed: No TCP Pose received!")
            return False

        # 2. Capture the 'Hit' position and lock it
        hit_pose = self.current_tcp_pose
        self.get_logger().info(f"Contact felt at Z: {hit_pose.position.z:.4f}")

        # 3. Create the ONE target (Hit Z + 4mm)
        target_pose = Pose()
        target_pose.position.x = hit_pose.position.x
        target_pose.position.y = hit_pose.position.y
        target_pose.position.z = hit_pose.position.z + distance_m
        target_pose.orientation = hit_pose.orientation

        self.get_logger().info(f"Moving to fixed Retract Z: {target_pose.position.z:.4f}")

        # 4. Use a synchronous Cartesian move to get there once
        return self.move_pose_cartesian_path(target_pose, speed=0.05)

    def smart_retract_python(self, distance_m=0.004, speed=0.01):
        """Fetches LIVE position via TF and lifts exactly distance_m."""
        from tf2_ros import TransformException
        try:
            # Look up the current live position of the tool relative to the base
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'tool0', now, 
                                                   timeout=rclpy.duration.Duration(seconds=1.0))
            
            current_z = trans.transform.translation.z
            target_z = current_z + distance_m
            
            self.get_logger().info(f"Felt contact at {current_z:.4f}. Lifting to {target_z:.4f}")
            
            target_pose = Pose()
            target_pose.position.x = trans.transform.translation.x
            target_pose.position.y = trans.transform.translation.y
            target_pose.position.z = target_z
            target_pose.orientation = trans.transform.rotation
            
            # Use Cartesian move for a perfectly straight vertical lift
            return self.move_pose_cartesian_path(target_pose)
        except Exception as e:
            self.get_logger().error(f"smart_retract_python failed: {e}")
            return False



def main(args=None):
    rclpy.init(args=args)
    planner = UR12eExtendedPlanner()
    
    # 1. Move to Home (Planned/Sync)
    planner.jmove_plan_sync(planner.home_seed,speed=0.05)
    
    # 2. Test Cartesian Planned Move (Sync)
    planner.get_logger().info("Executing move_plan_xyz_theta_sync...")
    planner.move_plan_xyz_theta_sync(0.2, 0.6, 0.5, math.radians(-45), speed=0.05)
    
    # 3. Test Cartesian Planned Move (Async)
    planner.get_logger().info("Starting move_plan_xyz_async...")
    f = planner.move_plan_xyz_async(0.2, 0.9, 0.5, speed=0.05)
    if f:
        rclpy.spin_until_future_complete(planner, f)
        planner.get_logger().info("Cartesian plan accepted and moving...")

    # 4. Test the Cartesian Path (Straight Line) to a target
    # Moving 20cm along the X-axis in a perfectly straight line
    target_x, target_y, target_z = 0.1, 0.8, 0.4
    planner.get_logger().info(f"Testing straight-line Cartesian path to: {target_x}, {target_y}, {target_z}")
    
    success = planner.move_xyz_cartesian_path(target_x, target_y, target_z)
    
    if success:
        planner.get_logger().info("Cartesian move successful!")
    else:
        planner.get_logger().error("Cartesian move failed or fell back to standard planning.")

    planner.get_logger().info(f"Testing straight-line Cartesian vertical path to: {target_x}, {target_y}, {target_z-0.1}, {45}")
    

    # Test a 10cm perfectly straight vertical descent at a 45-degree wrist angle
    success=planner.move_xyz_theta_cartesian_path(target_x, target_y, target_z - 0.1, math.radians(45), speed=0.2)
    if success:
        planner.get_logger().info("Cartesian move successful!")
    else:
        planner.get_logger().error("Cartesian move failed or fell back to standard planning.")


    # Define a test target pose
    test_pose = Pose()
    test_pose.position = Point(x=0.3, y=0.5, z=0.4)
    
    # Set orientation (pointing down, 45-degree twist)
    # Using the same math as your move_xyz_theta functions
    theta_rad = math.radians(45)
    test_pose.orientation.x = math.cos(theta_rad / 2.0)
    test_pose.orientation.y = -math.sin(theta_rad / 2.0)
    test_pose.orientation.z = 0.0
    test_pose.orientation.w = 0.0

    planner.get_logger().info("Testing move_pose_cartesian_path...")
    
    # Execute the move at a slow, controlled speed (5%)
    success = planner.move_pose_cartesian_path(test_pose, speed=0.05)
    
    if success:
        planner.get_logger().info("Pose Cartesian move completed successfully.")
    else:
        planner.get_logger().error("Pose Cartesian move failed.")

    # Test 1: Linear XYZ move at very slow speed (2%)
    planner.get_logger().info("Testing slow linear XYZ move...")
    planner.move_xyz_cartesian_path(0.3, 0.5, 0.4, speed=0.02)
    
    # Test 2: Pose Plan (curved path) to a specific rotation
    planner.get_logger().info("Testing Pose Plan move...")
    target_pose = Pose()
    target_pose.position = Point(x=0.1, y=0.8, z=0.5)
    # 90-degree wrist twist
    target_pose.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0) 
    planner.move_pose_plan_sync(target_pose, speed=0.1)


    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()