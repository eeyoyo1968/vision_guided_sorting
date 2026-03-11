import minimalmodbus
import time

def hard_force_open():
    try:
        gripper = minimalmodbus.Instrument('/tmp/ttyUR', 9)
        gripper.serial.baudrate = 115200
        gripper.serial.timeout = 0.5
        # Clear buffers to fix the "Too short response" error
        gripper.clear_buffers_before_each_transaction = True
        
        print("Forcing Deactivation (Dropping Tension)...")
        try:
            gripper.write_register(1000, 0x0000)
        except:
            print("Ignoring response error during deactivation...")
            
        time.sleep(2) # Give it time to physically relax

        print("Re-activating Gripper...")
        gripper.write_register(1000, 0x0100)
        time.sleep(2)

        print("Sending OPEN command...")
        # 0x0900 (Action), 3 (Position), 0x6432 (Speed/Force)
        gripper.write_registers(1000, [0x0900, 3, 0x6432])
        print("Success! Gripper should be open.")

    except Exception as e:
        print(f"Hard Reset Failed: {e}")

if __name__ == "__main__":
    hard_force_open()