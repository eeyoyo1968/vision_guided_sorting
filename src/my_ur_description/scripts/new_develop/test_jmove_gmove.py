import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math
import numpy as np

# Messages
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class UR12eController(Node):
    def __init__(self):
        super().__init__('ur12e_controller')
        
        # Action Clients
        self._arm_client = ActionClient(self, MoveGroup, 'move_action')
        # This matches the 'robotiq_85_left_knuckle_joint' in your list
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/robotiq_gripper_controller/follow_joint_trajectory')

        self.get_logger().info("Waiting for Action Servers...")
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info("Robot and Gripper Ready.")

    def jmove(self, jointvector):
        """Moves the UR12e Arm"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.start_state.is_diff = True 
        goal_msg.request.allowed_planning_time = 5.0
        
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        con = Constraints()
        for name, pos in zip(joint_names, jointvector):
            jc = JointConstraint(joint_name=name, position=float(pos), 
                                 tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            con.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(con)
        future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

    def gripper_move(self, pos):
        """Moves the Robotiq Gripper (0.0=Open, 0.8=Closed)"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['robotiq_85_left_knuckle_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [float(pos)]
        point.time_from_start.sec = 1
        
        goal_msg.trajectory.points.append(point)
        
        self.get_logger().info(f"Gripper to: {pos}")
        future = self._gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        res_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

def main():
    rclpy.init()
    bot = UR12eController()

    # Math Constants
    PI = math.pi
    D2R = PI / 180.0 # Multiplier for Degrees to Radians

    # Pose Definitions
    # Home: Using the -1.5707 you found worked
    #home = [0.0, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]
    home = [3.1415, -0.7854, -2.3562, -1.5707, 1.5707, 0.0]
        
    # Pick positions using math for clarity
    pick = np.array([-16.0, -74.0, 113.0, -129.0, -90.0, 74.0]) * D2R
    pick2 = np.array([-16.0, -72.0, 113.0, -129.0, -90.0, 86.0]) * D2R

    try:
        # Step 1: Initialize
        bot.gripper_move(0.0) # Open
        bot.jmove(home)

        # Step 2: Go to Pick
        bot.jmove(pick)
        bot.jmove(pick2)
        
        # Step 3: Close Gripper
        bot.gripper_move(0.5) # 0.5 is usually a good partial close for testing
        time.sleep(0.5)

        # Step 4: Return
        bot.jmove(home)
        bot.gripper_move(0.0) # Open

    finally:
        bot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()