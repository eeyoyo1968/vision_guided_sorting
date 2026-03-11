import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

class UR12eCustomHomeMover(Node):
    def __init__(self):
        super().__init__('ur12e_custom_mover')
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.current_joints = None
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

    def joint_cb(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        # Filter to ensure we only save state when all arm joints are present
        if all(name in name_to_pos for name in self.joint_names):
            self.current_joints = [name_to_pos[name] for name in self.joint_names]

    def send_slow_home_goal(self):
        if self.current_joints is None:
            self.get_logger().warn('Still waiting for a complete JointState message...')
            return None

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Action server not available!')
            return None

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        # Point 1: Current physical position
        start_point = JointTrajectoryPoint()
        start_point.positions = self.current_joints
        start_point.time_from_start.sec = 0
        
        # Point 2: Your Custom Home positions
        home_point = JointTrajectoryPoint()
        home_point.positions = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]
        home_point.time_from_start.sec = 10  # Slow and safe 10s move

        goal_msg.trajectory.points = [start_point, home_point]
        
        self.get_logger().info('Sending command to move to Custom Home...')
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    mover = UR12eCustomHomeMover()
    
    # Spin to gather current joint states
    import time
    start_t = time.time()
    while rclpy.ok() and mover.current_joints is None and (time.time() - start_t) < 5.0:
        rclpy.spin_once(mover, timeout_sec=0.1)
    
    future = mover.send_slow_home_goal()
    
    if future:
        rclpy.spin_until_future_complete(mover, future)
        # Fixed the typo here: replaced self with mover
        mover.get_logger().info('Goal accepted by the UR controller!')
    
    # Wait a moment for the action to actually start
    time.sleep(1.0)
    
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()