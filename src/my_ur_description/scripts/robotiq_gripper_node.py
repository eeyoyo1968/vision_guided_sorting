import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from example_interfaces.srv import SetInt32 # Simple ROS2 service
import minimalmodbus

class RobotiqGripperNode(Node):
    def __init__(self):
        super().__init__('robotiq_gripper_node')
        
        # 1. Setup Serial
        try:
            self.gripper = minimalmodbus.Instrument('/tmp/ttyUR', 9)
            self.gripper.serial.baudrate = 115200
            self.gripper.serial.timeout = 0.5
            self.get_logger().info("Gripper Hardware Initialized.")
        except Exception as e:
            self.get_logger().error(f"Failed to open gripper serial: {e}")

        # 2. The Service: Other nodes call this to move the gripper
        self.srv = self.create_service(SetInt32, 'set_gripper_pos', self.set_pos_callback)
        
        # 3. The Heartbeat: Run every 1 second to keep the instrument active
        self.heartbeat_timer = self.create_timer(1.0, self.maintain_heartbeat)

    def set_pos_callback(self, request, response):
        try:
            # Command: [Action, Position, Speed/Force]
            self.gripper.write_registers(1000, [0x0900, request.data, 0x6432])
            response.success = True
            self.get_logger().info(f"Gripper moved to: {request.data}")
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Gripper write failed: {e}")
        return response

    def maintain_heartbeat(self):
        """Requests gripper status to keep the Modbus connection 'warm'."""
        try:
            # Read a register just to maintain communication
            self.gripper.read_register(2000) 
        except Exception:
            pass # Silent failure for heartbeat

def main():
    rclpy.init()
    node = RobotiqGripperNode()
    rclpy.spin(node)
    rclpy.shutdown()