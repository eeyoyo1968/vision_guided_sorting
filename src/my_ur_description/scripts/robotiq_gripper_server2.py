import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import minimalmodbus
import time

class RobotiqGripperServer(Node):
    def __init__(self):
        super().__init__('robotiq_gripper_server')
        
        try:
            self.gripper = minimalmodbus.Instrument('/tmp/ttyUR', 9)
            self.gripper.serial.baudrate = 115200
            self.gripper.serial.timeout = 0.5
    
            self.get_logger().info("--- Initializing Gripper Hardware ---")
            # 1. Clear faults/Reset
            self.gripper.write_register(1000, 0x0000)
            time.sleep(0.5)
            
            # 2. Activate (Requires time for the sweep)
            self.gripper.write_register(1000, 0x0100)
            self.get_logger().info("Waiting 4s for Activation Sweep...")
            time.sleep(4.0) 
            
            self.get_logger().info("Robotiq Gripper Server Online and Blue.")
        except Exception as e:
            self.get_logger().error(f"Hardware Error: {e}")

        self.sub = self.create_subscription(Int32, 'gripper_command', self.listener_callback, 10)
        self.timer = self.create_timer(1.0, self.maintain_heartbeat)

    def listener_callback(self, msg):
        try:
            pos = msg.data
            speed = 100
            force = 50
            # Use the exact 3-register payload from your working test script
            payload = [0x0900, pos, (speed << 8) | force]
            self.get_logger().info(f"Moving to: {pos}")
            self.gripper.write_registers(1000, payload)
        except Exception as e:
            self.get_logger().error(f"Failed to write to gripper: {e}")

    def maintain_heartbeat(self):
        try:
            # Heartbeat ensures the RS485 connection stays alive
            self.gripper.read_register(2000) 
        except Exception:
            pass

def main():
    rclpy.init()
    node = RobotiqGripperServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()