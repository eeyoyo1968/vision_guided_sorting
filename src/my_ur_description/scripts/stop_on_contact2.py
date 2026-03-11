import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class URScriptForceTester(Node):
    def __init__(self):
        super().__init__('ur_script_force_tester')
        # Using your confirmed topic
        self.script_pub = self.create_publisher(String, '/urscript_interface/script_command', 10)

    def run_test(self):
        self.get_logger().info("INITIATING DESCENT... Prepare to trigger I/O in URSim!")
        
        # This script tells the robot: 
        # 1. Start moving towards a point 20cm below.
        # 2. IF Digital Input 0 becomes TRUE, stop immediately.
        # 3. Move back up 5cm.
        script = (
            "def test_contact():\n"
            "  global p_start = get_actual_tcp_pose()\n"
            "  global p_target = pose_add(p_start, p[0,0,-0.2,0,0,0])\n"
            "  # We use a while loop to simulate a 'stop on condition'\n"
            "  # You can toggle Digital Input 0 in URSim to trigger this\n"
            "  movel(p_target, a=0.2, v=0.02)\n"
            "  while (get_digital_in(0) == False):\n"
            "    sync()\n"
            "  end\n"
            "  stopl(2.0)\n"
            "  textmsg(\"CONTACT DETECTED - RETRACTING\")\n"
            "  movel(pose_add(get_actual_tcp_pose(), p[0,0,0.05,0,0,0]), a=0.5, v=0.1)\n"
            "end\n"
            "test_contact()\n"
        )
        
        msg = String()
        msg.data = script
        time.sleep(1.0) # Ensure publisher is ready
        self.script_pub.publish(msg)

def main():
    rclpy.init()
    node = URScriptForceTester()
    node.run_test()
    # Keep alive long enough to send
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()