import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32 # Use standard Int32
import time
import collections
import numpy as np
import math

from move_plan_xyz_theta import UR12eExtendedPlanner

class VisionSortingBrainOnce(UR12eExtendedPlanner):
    def __init__(self):
        super().__init__()
        self.pose_buffer = collections.deque(maxlen=10)
        self.is_busy = False
        self.current_class = ""
        self.latest_pose = None
        
        self.GRIPPER_OPEN = 3
        self.GRIPPER_CLOSED = 228 
        self.D2R = math.pi / 180.0

        # 1. Setup Gripper Publisher
        self.gripper_pub = self.create_publisher(Int32, 'gripper_command', 10)

        self.obj_pose_sub = self.create_subscription(PoseStamped, '/grasp/pose', self.pose_cb, 10)
        self.obj_class_sub = self.create_subscription(String, '/grasp/class', self.class_cb, 10)

    def set_gripper(self, position):
        """Publishes the position and waits slightly for action."""
        msg = Int32()
        msg.data = int(position)
        self.gripper_pub.publish(msg)
        time.sleep(0.5) # Small buffer for the serial command to execute

    def pose_cb(self, msg):
        if not self.is_busy:
            self.latest_pose = msg.pose

    def class_cb(self, msg):
        if not self.is_busy:
            self.current_class = msg.data

    def check_stability(self):
        if self.latest_pose is None: return False
        self.pose_buffer.append([self.latest_pose.position.x, self.latest_pose.position.y])
        if len(self.pose_buffer) < 10: return False
        arr = np.array(self.pose_buffer)
        spread = np.max(arr, axis=0) - np.min(arr, axis=0)
        return all(spread < 0.015)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Converts euler angles to (qx, qy, qz, qw)."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]

    def execute_sorting_cycle(self):
        self.is_busy = True
        p = self.latest_pose
        
        x_target = -0.03 - p.position.x
        y_target = 1.28 - p.position.y
        z_pick = max(0.232, 0.20 - 0.02 + p.position.z)
        
        
        q = p.orientation
        theta = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        angle_rad = -theta + (math.pi/2)

        # 2. Orientation Logic (Top-down Pick)
        # Convert your theta/angle_rad into a quaternion
        # This creates a 'Gripper Pointing Down' orientation with Z-rotation
        q_target = self.euler_to_quaternion(math.pi, 0, angle_rad) # Using a helper




        self.get_logger().info(f"Target: {self.current_class}. Moving...")

        try:
            self.set_gripper(self.GRIPPER_OPEN)

                        # 1. Approach to safe height (10cm above)
            self.move_plan_xyz_theta_sync(x_target, y_target, z_pick + 0.1, angle_rad, speed=0.1)
            
            # 1. Approach to 3cm above target
            #self.move_plan_xyz_theta_sync(x_target, y_target, z_pick + 0.01, angle_rad, speed=0.02)
            self.move_xyz_theta_cartesian_sync(x_target, y_target, z_pick + 0.01, angle_rad, speed=0.02)

            if z_pick < 0.28:
                self.get_logger().info("Executing Smart Async Touch...")
                
                # FIX: Pass all quaternion components to get_ik_solution
                floor_z = z_pick - 0.02
                #floor_joints = self.get_ik_solution(
                #    x_target, y_target, floor_z,
                #    q_target[0], q_target[1], q_target[2], q_target[3]
                #)
                #floor_joints = self.get_ik_solution(
                #    x_target, y_target, floor_z,
                #    q.x, q.y, q.z, q.w
                #)
                
                #if floor_joints:
                    #self.smart_descent_skill(floor_joints, duration=3.0)
                self.smart_descent_skill_xyz_theta(x_target,y_target, floor_z, angle_rad, duration=3.0)
                
                # Retract and Grip
                self.move_plan_xyz_theta_sync(x_target, y_target, z_pick + 0.008, angle_rad, speed=0.02)
                self.set_gripper(self.GRIPPER_CLOSED)
                time.sleep(1.0)
            else:
                # Normal pick for tall objects
                self.move_plan_xyz_theta_sync(x_target, y_target, z_pick, angle_rad, speed=0.02)
                self.set_gripper(self.GRIPPER_CLOSED)
            
            # Lift
            self.move_plan_xyz_theta_sync(x_target, y_target, z_pick + 0.15, angle_rad, speed=0.1)
            
            # Sort
            if self.current_class == "glove":
                bin_soft = np.array([270.0, -54.27, -145.26, -70.48, 90.0, 0.0]) * self.D2R
                self.jmove_plan_sync(bin_soft)
            else:
                # Using your preferred bin_hard_above
                bin_hard_above = np.array([350.0, -90.0, -121.65, -56.14, 90.0, 80.5]) * self.D2R
                self.jmove_plan_sync(bin_hard_above)
                bin_hard = np.array([350.0, -100.0, -128.0, -41.5, 90.0, 81.0]) * self.D2R
                self.jmove_plan_sync(bin_hard)

            self.set_gripper(self.GRIPPER_OPEN)
            time.sleep(0.5)
            
            # Home point: [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]
            self.jmove_plan_sync(self.home_seed)
            return True

        except Exception as e:
            self.get_logger().error(f"Cycle failed: {e}")
            return False
        finally:
            self.is_busy = False

def main(args=None):
    rclpy.init(args=args)
    node = VisionSortingBrainOnce()
    try:
        done = False
        while rclpy.ok() and not done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.is_busy and node.check_stability():
                if node.execute_sorting_cycle():
                    done = True
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()