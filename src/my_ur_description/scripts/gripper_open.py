import minimalmodbus
import time

# Configuration
PORT = '/tmp/ttyUR'
SLAVE_ADDRESS = 9

def open_gripper():
    try:
        # Initialize instrument
        gripper = minimalmodbus.Instrument(PORT, SLAVE_ADDRESS)
        gripper.serial.baudrate = 115200
        gripper.serial.timeout = 0.2
        
        print("Activating Gripper...")
        # Register 1000: 0x0100 activates the gripper
        gripper.write_register(1000, 0x0100)
        time.sleep(1) # Wait for activation
        
        print("Opening Gripper...")
        # Register 1000: 0x0900 (Action), Register 1001: 3 (Pos), Register 1002: Speed/Force
        # We send these as a block of 3 registers
        gripper.write_registers(1000, [0x0900, 3, 0x6432])
        
        print("Done.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    open_gripper()