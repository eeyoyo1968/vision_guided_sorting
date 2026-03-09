#!/usr/bin/env python3
"""
ROS2 Action Server for Robotiq Gripper in URSim
Bridges ROS2 gripper commands to URSim via URScript
Works with URCap installed in URSim
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import socket
import time
import threading

class RobotiqURSimBridge(Node):
    def __init__(self):
        super().__init__('robotiq_ursim_bridge')
        
        # Parameters
        self.robot_ip = self.declare_parameter('robot_ip', '127.0.0.1').value
        self.script_port = 30002  # URScript interface
        self.rtde_port = 30004    # Real-time data exchange
        
        # Gripper state
        self.current_position = 0.0  # 0.0 = open, 0.8 = closed (radians)
        self.gripper_moving = False
        
        # Publisher for gripper state
        self.state_pub = self.create_publisher(
            JointState,
            '/robotiq_gripper_state',
            10
        )
        
        # Action server matching your existing controller
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/robotiq_gripper_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        # Start state publisher
        self.timer = self.create_timer(0.1, self.publish_state)
        
        self.get_logger().info(f"Robotiq URSim Bridge ready (URSim IP: {self.robot_ip})")
    
    def send_urscript(self, script):
        """Send URScript commands to URSim."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect((self.robot_ip, self.script_port))
            
            # Send script
            s.send(script.encode())
            s.close()
            
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send URScript: {e}")
            return False
    
    def gripper_command(self, position_rad):
        """
        Send gripper command to URSim via URCap.
        position_rad: 0.0 (open) to 0.8 (closed)
        """
        # Convert radians to gripper position (0-255)
        # 0.0 rad = fully open = 0
        # 0.8 rad = fully closed = 255
        gripper_pos = int((position_rad / 0.8) * 255)
        gripper_pos = max(0, min(255, gripper_pos))
        
        self.get_logger().info(f"Commanding gripper to position: {gripper_pos}/255 ({position_rad:.3f} rad)")
        
        # URScript for Robotiq URCap
        # This varies by URCap version - adjust as needed
        ur_script = f"""
def gripper_move():
    # Activate gripper if not already active
    rq_activate()
    sleep(0.5)
    
    # Move to position
    # rq_move_and_wait(pos, speed, force)
    rq_move_and_wait({gripper_pos}, 255, 255)
end

gripper_move()
"""
        
        success = self.send_urscript(ur_script)
        
        if success:
            # Update internal state
            self.current_position = position_rad
        
        return success
    
    def execute_callback(self, goal_handle):
        """Execute gripper trajectory action."""
        request = goal_handle.request
        
        # Extract target position from trajectory
        if len(request.trajectory.points) == 0:
            self.get_logger().error("Empty trajectory received")
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        
        # Get final point
        target_point = request.trajectory.points[-1]
        target_position = target_point.positions[0]  # First joint
        
        self.get_logger().info(f"Executing gripper move to {target_position:.3f} rad")
        
        # Mark as moving
        self.gripper_moving = True
        
        # Send command
        success = self.gripper_command(target_position)
        
        if not success:
            goal_handle.abort()
            self.gripper_moving = False
            return FollowJointTrajectory.Result()
        
        # Wait for movement (simulated timing)
        duration = target_point.time_from_start.sec + target_point.time_from_start.nanosec / 1e9
        time.sleep(max(0.5, duration))
        
        # Mark as complete
        self.gripper_moving = False
        goal_handle.succeed()
        
        result = FollowJointTrajectory.Result()
        return result
    
    def publish_state(self):
        """Publish current gripper state."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['robotiq_85_left_knuckle_joint']
        msg.position = [self.current_position]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        
        self.state_pub.publish(msg)

def main():
    rclpy.init()
    bridge = RobotiqURSimBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()