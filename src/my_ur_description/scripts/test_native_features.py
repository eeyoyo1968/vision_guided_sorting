import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from controller_manager_msgs.srv import SwitchController, ListControllers
import time

class URNativeFeatureTester(Node):
    def __init__(self):
        super().__init__('ur_native_feature_tester')
        
        # 1. URScript Publisher
        # Note: Your topic list showed '/urscript_interface/script_command'
        self.script_pub = self.create_publisher(String, '/urscript_interface/script_command', 10)
        
        # 2. Controller Management Clients
        self.list_ctrl_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.switch_ctrl_client = self.create_client(SwitchController, '/controller_manager/switch_controller')

    def test_urscript_blink(self):
        """Tests the URScript interface by asking the robot to log a message and move slightly."""
        self.get_logger().info("Testing URScript interface...")
        msg = String()
        # This script simply sends a popup to the Teach Pendant and moves the robot 1cm up
        msg.data = (
            "def test_interface():\n"
            "  textmsg(\"ROS2 URScript Connection Active\")\n"
            "  p_now = get_actual_tcp_pose()\n"
            "  p_up = pose_add(p_now, p[0,0,0.01,0,0,0])\n"
            "  movel(p_up, a=1.2, v=0.05)\n"
            "end\n"
            "test_interface()\n"
        )
        self.script_pub.publish(msg)
        self.get_logger().info("URScript sent. Check URSim/Teach Pendant for 'ROS2 URScript Connection Active'.")

    def activate_contact_controller(self):
        """Attempts to switch to tool_contact_controller for ROS 2 Humble."""
        if not self.switch_ctrl_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Controller Manager service not available")
            return False

        req = SwitchController.Request()
        # In Humble, we use activate_controllers and deactivate_controllers
        req.activate_controllers = ['tool_contact_controller']
        req.deactivate_controllers = ['scaled_joint_trajectory_controller']
        
        # Use STRICT to ensure it only switches if safe, 
        # or BEST_EFFORT if you want it to try regardless of other controller states
        req.strictness = SwitchController.Request.STRICT 
        
        # Note: activate_as_group is removed as it's not in Humble
        
        self.get_logger().info("Attempting to activate tool_contact_controller...")
        future = self.switch_ctrl_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        if res and res.ok:
            self.get_logger().info("SUCCESS: tool_contact_controller is now ACTIVE.")
            return True
        else:
            self.get_logger().error("FAILED: Switch rejected. Ensure URSim is in REMOTE mode and Program is PLAYING.")
            return False
        
        
def main():
    rclpy.init()
    tester = URNativeFeatureTester()
    
    # Test 1: URScript
    tester.test_urscript_blink()
    time.sleep(2.0)
    
    # Test 2: Controller Switching
    tester.activate_contact_controller()
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()