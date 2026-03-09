import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionIKRequest
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

        # Use your specific topic from the list
        self.wrench_sub = self.create_subscription(
            WrenchStamped, 
            '/UR20255101361/tcp_wrench', 
            self.wrench_cb, 
            10
        )


        # State tracking
        self.current_joints = None
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        # Reference Home for IK Seed
        self.home_seed = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]
        
        self.get_logger().info("UR12e Extended Planner Initialized")


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

    def jmove_plan_sync(self, joint_vector):
        """Planned move with collision avoidance (Sync)."""
        future = self.jmove_plan_async(joint_vector)
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

    def move_plan_xyz_async(self, x, y, z, frame_id="base_link"):
        """Asynchronous planned move to XYZ (Pointing Straight Down)."""
        return self.move_plan_xyz_theta_async(x, y, z, 0.0, frame_id)

    def move_plan_xyz_sync(self, x, y, z, frame_id="base_link"):
        """Synchronous planned move to XYZ (Pointing Straight Down)."""
        return self.move_plan_xyz_theta_sync(x, y, z, 0.0, frame_id)
    
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
        """Calls MoveIt service with safety checks."""
        if not self._cartesian_client.wait_for_service(timeout_sec=2.0):
            return None

        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = "ur_manipulator"
        req.waypoints = waypoints
        req.max_step = 0.01 
        req.jump_threshold = 0.0 # Strict jump check
        req.avoid_collisions = True

        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        # ONLY proceed if 100% of the path is possible (fraction == 1.0)
        if res and res.fraction >= 1.0:
            return res.solution
        else:
            percent = (res.fraction * 100) if res else 0
            self.get_logger().error(f"Cartesian Path Failed! Only {percent:.1f}% possible. Avoiding move.")
            return None
        
    def move_pose_cartesian_path(self, pose):
        """Executes a straight-line move to a specific geometry_msgs/Pose."""
        return self.execute_cartesian_trajectory(self.plan_cartesian_path([pose]))

    def move_xyz_cartesian_path(self, x, y, z):
        """Straight line move with fallback."""
        target = Pose()
        target.position = Point(x=float(x), y=float(y), z=float(z))
        # This keeps the gripper pointing down
        target.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
        
        plan = self.plan_cartesian_path([target])
        if plan:
            return self.execute_cartesian_trajectory(plan)
        else:
            # FALLBACK: If straight line fails, use a standard curved plan for safety
            self.get_logger().warn("Falling back to standard jmove_plan for safety.")
            return self.move_plan_xyz_theta_sync(x, y, z, 0.0)
        
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


def main(args=None):
    rclpy.init(args=args)
    planner = UR12eExtendedPlanner()
    
    # 1. Move to Home (Planned/Sync)
    planner.jmove_plan_sync(planner.home_seed)
    
    # 2. Test Cartesian Planned Move (Sync)
    planner.get_logger().info("Executing move_plan_xyz_theta_sync...")
    planner.move_plan_xyz_theta_sync(0.2, 0.6, 0.5, math.radians(-45))
    
    # 3. Test Cartesian Planned Move (Async)
    planner.get_logger().info("Starting move_plan_xyz_async...")
    f = planner.move_plan_xyz_async(0.2, 0.4, 0.3)
    if f:
        rclpy.spin_until_future_complete(planner, f)
        planner.get_logger().info("Cartesian plan accepted and moving...")

    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()