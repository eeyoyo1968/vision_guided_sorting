import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import minimalmodbus
import time

class RobotiqGripperServer(Node):
    def __init__(self):
        super().__init__('robotiq_gripper_server')
        
        # 1. Hardware Initialization
        try:
            self.gripper = minimalmodbus.Instrument('/tmp/ttyUR', 9)
            self.gripper.serial.baudrate = 115200
            self.gripper.serial.timeout = 0.5
    
            # Robust Activation: Clear old faults, then Activate
            self.get_logger().info("Resetting and Activating Gripper...")
            self.gripper.write_register(1000, 0x0000) # Clear
            time.sleep(0.1)
            self.gripper.write_register(1000, 0x0100) # Activate (rACT = 1)
            self.get_logger().info("Robotiq Gripper Server Online.")
        except Exception as e:
            self.get_logger().error(f"Hardware Error: {e}")


        # 2. Topic Subscriber: No custom service files needed!
        self.sub = self.create_subscription(Int32, 'gripper_command', self.listener_callback, 10)
        
        # 3. Heartbeat Timer (1Hz)
        self.timer = self.create_timer(1.0, self.maintain_heartbeat)

    def listener_callback(self, msg):
        try:
            self.get_logger().info(f"Moving gripper to: {msg.data}")
            # rACT=1, rGTO=1, rPR=msg.data, rSP=100, rFR=50
            self.gripper.write_registers(1000, [0x0900, msg.data, 0x6432])
        except Exception as e:
            self.get_logger().error(f"Failed to write to gripper: {e}")

    def maintain_heartbeat(self):
        try:
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