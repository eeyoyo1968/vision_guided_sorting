import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class URScriptContactTester(Node):
    def __init__(self):
        super().__init__('ur_script_contact_tester')
        # Based on your topic list: /urscript_interface/script_command
        self.script_pub = self.create_publisher(String, '/urscript_interface/script_command', 10)

    def test_stop_on_contact(self):
        self.get_logger().info("Sending Stop-on-Contact URScript...")
        # This script moves the robot down 10cm or until 5 Newtons of force is felt
        # Since it's URSim, you'll need to manually trigger an I/O or Force in the GUI
        script = (
            "def contact_test():\n"
            "  global p_start = get_actual_tcp_pose()\n"
            "  global p_target = pose_add(p_start, p[0,0,-0.1,0,0,0])\n"
            "  # stop_on_contact() stops the move if force > threshold\n"
            "  stop_on_contact()\n"
            "  movel(p_target, a=0.1, v=0.02)\n"
            "  # Move back up slightly after contact\n"
            "  movel(pose_add(get_actual_tcp_pose(), p[0,0,0.02,0,0,0]), a=0.5, v=0.1)\n"
            "end\n"
            "contact_test()\n"
        )
        msg = String()
        msg.data = script
        self.script_pub.publish(msg)

def main():
    rclpy.init()
    tester = URScriptContactTester()
    tester.test_stop_on_contact()
    rclpy.shutdown()

if __name__ == '__main__':
    main()