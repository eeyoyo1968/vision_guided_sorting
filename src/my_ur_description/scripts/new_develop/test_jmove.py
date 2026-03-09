import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import numpy as np

class UR12eJointStepper(Node):
    def __init__(self):
        super().__init__('ur12e_joint_stepper')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info("Connecting to MoveGroup...")
        self._action_client.wait_for_server()

    def jmove(self, jointvector):
        self.get_logger().info(f"Moving to: {jointvector}")
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        # FIX 1: Explicitly tell MoveIt to start from the current actual position
        goal_msg.request.start_state.is_diff = True 
        
        # FIX 2: Increase planning time and velocity for the last move
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        con = Constraints()
        for name, pos in zip(joint_names, jointvector):
            jc = JointConstraint(joint_name=name, position=float(pos), 
                                 tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            con.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(con)

        # FIX 3: Wait a split second for the buffers to clear
        time.sleep(0.5)

        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        
        if not handle.accepted:
            return False

        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

def main():
    rclpy.init()
    bot = UR12eJointStepper()

    pi=3.14159
    
    # 1. Define your vectors
    # Home: Elbow Up configuration
    #home = [0.0, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]
    home = [3.1415, -0.7854, -2.3562, -1.5707, 1.5707, 0.0]
    
    # Pick Approach: Elbow Down configuration
    pick_approach = [0.0, -1.5707, 1.5707, -1.5707, -1.5707, 0.0]

    # Pick -16,-74, 113, -129, -90, 74
    pick = np.array([-16.0,-74.0, 113.0, -129.0, -90.0, 74.0])*pi/180

    # Pick2 -16,-69, 113, -134, -90, 86
    pick2 = np.array([-16,-72, 113, -129, -90, 86])*pi/180

    # Standard UR "Up" pose: All zeros except shoulder lift and elbow
    up_pose = [-1.5707, -1.5707, 0.0, -1.5707, 0.0, 0.0]
 
    
    # 2. Execute Sequence
    #bot.get_logger().info("Step 1: Moving to HOME")
    #bot.jmove(home)
    #time.sleep(5.0)

    bot.get_logger().info("Step 1: Moving to HOME")
    bot.jmove(home)
    time.sleep(1.0)

    bot.get_logger().info("Step 2: Moving to PICK_APPROACH")
    bot.jmove(pick_approach)
    time.sleep(1.0)

    bot.get_logger().info("Step 3: Moving to PICK")
    bot.jmove(pick)
    time.sleep(1.0)

    bot.get_logger().info("Step 4 Moving to PICK2")
    bot.jmove(pick2)
    time.sleep(1.0)

    bot.get_logger().info("Step 1: Moving to UP")
    bot.jmove(up_pose)
    time.sleep(1.0)

    bot.get_logger().info("Step 2: Moving to PICK_APPROACH")
    bot.jmove(pick_approach)
    time.sleep(1.0)

    bot.get_logger().info("Step 3: Moving to PICK")
    bot.jmove(pick)
    time.sleep(1.0)

    bot.get_logger().info("Step 4 Moving to PICK2")
    bot.jmove(pick2)
    time.sleep(1.0)

    bot.get_logger().info("Step 1: Moving to HOME")
    bot.jmove(home)
    time.sleep(1.0)

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()