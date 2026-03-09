import minimalmodbus
import time

# Configuration
PORT = '/tmp/ttyUR'
SLAVE_ADDRESS = 9

def close_gripper():
    try:
        gripper = minimalmodbus.Instrument(PORT, SLAVE_ADDRESS)
        gripper.serial.baudrate = 115200
        gripper.serial.timeout = 0.2
        
        print("Activating Gripper...")
        gripper.write_register(1000, 0x0100)
        time.sleep(1)
        
        print("Closing Gripper...")
        # Position 228 is your verified closed limit
        gripper.write_registers(1000, [0x0900, 228, 0x6432])
        
        print("Done.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    close_gripper()