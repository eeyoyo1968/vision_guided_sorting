import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import collections
import numpy as np
import math
import minimalmodbus 

# Import your Extended Planner class
from move_plan_xyz_theta import UR12eExtendedPlanner

class VisionSortingBrainOnce(UR12eExtendedPlanner):
    def __init__(self):
        super().__init__()
        self.pose_buffer = collections.deque(maxlen=10)
        self.is_busy = False
        self.current_class = ""
        self.latest_pose = None
        self.gripper_connected = False

        self.GRIPPER_OPEN = 3
        self.GRIPPER_CLOSED = 228 
        self.D2R = math.pi / 180.0

        # Gripper Initialization
        try:
            self.gripper = minimalmodbus.Instrument('/tmp/ttyUR', 9)
            self.gripper.serial.baudrate = 115200
            self.gripper.serial.timeout = 0.2
            self.gripper.write_register(1000, 0x0100) 
            self.gripper_connected = True
            self.get_logger().info("Robotiq Gripper Connected.")
        except Exception:
            self.get_logger().error("Gripper Connection Failed.")

        self.obj_pose_sub = self.create_subscription(PoseStamped, '/grasp/pose', self.pose_cb, 10)
        self.obj_class_sub = self.create_subscription(String, '/grasp/class', self.class_cb, 10)

    def pose_cb(self, msg):
        if not self.is_busy:
            self.latest_pose = msg.pose

    def class_cb(self, msg):
        if not self.is_busy:
            self.current_class = msg.data

    def set_gripper(self, position):
        if self.gripper_connected:
            try:
                self.gripper.write_registers(1000, [0x0900, position, 0x6432])
            except Exception: pass

    def check_stability(self):
        if self.latest_pose is None: return False
        self.pose_buffer.append([self.latest_pose.position.x, self.latest_pose.position.y])
        if len(self.pose_buffer) < 10: return False
        arr = np.array(self.pose_buffer)
        spread = np.max(arr, axis=0) - np.min(arr, axis=0)
        return all(spread < 0.015)

    def execute_sorting_cycle(self):
        self.is_busy = True
        p = self.latest_pose
        
        # Coordinate math
        x_target = -0.03 - p.position.x
        y_target = 1.28 - p.position.y
        z_pick = max(0.232, 0.20 - 0.02 + p.position.z)
        
        # Orientation
        q = p.orientation
        theta = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        angle_rad = -theta + (math.pi/2)

        self.get_logger().info(f"Target Acquired: {self.current_class}. Starting Pick & Sort...")

        try:
            self.set_gripper(self.GRIPPER_OPEN)
            
            # 1. Approach and Pick (Planned moves)
            self.move_plan_xyz_theta_sync(x_target, y_target, z_pick + 0.1, angle_rad)
            self.move_plan_xyz_theta_sync(x_target, y_target, z_pick, angle_rad)
            
            self.set_gripper(self.GRIPPER_CLOSED)
            time.sleep(1.0)
            
            # 2. Lift
            self.move_plan_xyz_theta_sync(x_target, y_target, z_pick + 0.15, angle_rad)
            
            # 3. Sort to Bins
            if self.current_class == "glove":
                bin_soft = np.array([270.0, -54.27, -145.26, -70.48, 90.0, 0.0]) * self.D2R
                self.jmove_plan_sync(bin_soft)
            else:
                # Use your optimized above position
                self.get_logger().info("Moving to bin_hard_above...")
                bin_hard_above = np.array([350.0, -90.0, -121.65, -56.14, 90.0, 80.5]) * self.D2R
                self.jmove_plan_sync(bin_hard_above)
                
                # Move to actual drop position
                bin_hard = np.array([350.0, -100.0, -128.0, -41.5, 90.0, 81.0]) * self.D2R
                self.jmove_plan_sync(bin_hard)

            self.set_gripper(self.GRIPPER_OPEN)
            time.sleep(0.5)
            
            # 4. Final Return to Home
            self.jmove_plan_sync(self.home_seed)
            self.get_logger().info("Cycle complete. Robot returned to home.")
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
        
        node.get_logger().info("Exiting vision guided sorting.")
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()