import minimalmodbus
import time

instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 1.0

def move_gripper_final(pos, speed=100, force=50):
    """
    Corrected Register Mapping:
    Reg 1000: 0x0900 (Action 09, Reserved 00)
    Reg 1001: pos    (Reserved 00, Position XX) -> Just 'pos' since it's the low byte
    Reg 1002: (speed << 8) | force
    """
    try:
        # We only send 3 registers (1000, 1001, 1002)
        payload = [
            0x0900,                # Action: Activate + GoTo
            pos,                   # Position: (0-255)
            (speed << 8) | force   # Speed and Force
        ]
        
        print(f"Sending Command -> Pos: {pos}, Spd: {speed}, For: {force}")
        instrument.write_registers(1000, payload)
        
        # Heartbeat/Wait for motion
        for _ in range(10):
            # Read status from 2000 to see if it moves
            status = instrument.read_registers(2000, 1)[0]
            gOBJ = (status >> 14) & 0x03 # Object status
            if gOBJ == 0x03: # Done
                print("  Reached Target.")
                break
            time.sleep(0.3)
            
    except Exception as e:
        print(f"Error: {e}")

def run():
    print("--- Initializing ---")
    instrument.write_register(1000, 0x0100) # Activate
    time.sleep(4) # Wait for calibration sweep
    
    # THE TEST
    print("--- Executing Movements ---")
    move_gripper_final(255) # Close
    time.sleep(1)
    move_gripper_final(0)   # Open
    time.sleep(1)
    move_gripper_final(128) # Halfway

if __name__ == "__main__":
    run()