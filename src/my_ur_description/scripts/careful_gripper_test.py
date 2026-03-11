import minimalmodbus
import time

# Communication Setup
instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 0.5

def get_detailed_feedback():
    """Reads the actual state from the 2000-series registers."""
    try:
        # Read 3 registers starting at 2000
        regs = instrument.read_registers(2000, 3)
        actual_pos = (regs[2] >> 8) & 0xFF
        current = regs[2] & 0xFF
        # gOBJ: 0=Moving, 1=Outer Object, 2=Inner Object, 3=At Position
        status_byte = (regs[0] >> 8) & 0xFF
        obj_status = (status_byte >> 6) & 0x03 
        
        return actual_pos, current, obj_status
    except:
        return None, None, None

def careful_move(target_pos, label, speed=100, force=50):
    print(f"\n--- Testing: {label} (Target: {target_pos}) ---")
    
    # Send Command (Corrected 3-register mapping)
    # [Action, Position, Speed/Force]
    payload = [0x0900, target_pos, (speed << 8) | force]
    instrument.write_registers(1000, payload)
    
    # Monitor the movement for up to 3 seconds
    start_time = time.time()
    while time.time() - start_time < 3.0:
        pos, cur, obj = get_detailed_feedback()
        if pos is not None:
            print(f"  Current Pos: {pos} | Motor: {cur*10}mA | Status: {obj}", end='\r')
            if obj >= 1: # 1, 2, or 3 means motion stopped
                print(f"\n  Result: Stopped at {pos}")
                break
        time.sleep(0.1)

def run_diagnostic():
    print("Initializing Careful Test...")
    # 1. Clear faults and Activate
    instrument.write_register(1000, 0x0000)
    time.sleep(0.5)
    instrument.write_register(1000, 0x0100)
    print("Waiting 4s for Activation Sweep (Fingers will move)...")
    time.sleep(4.0)

    # 2. Careful Sequence
    # Test 1: Full Open
    careful_move(0, "Full Open")
    
    # Test 2: Mid-Point (Verify distance control)
    careful_move(128, "Halfway Point")
    
    # Test 3: Full Close (Watch motor current)
    careful_move(255, "Full Close")
    
    # Test 4: Back to Ready
    careful_move(0, "Return to Ready")

    print("\nDiagnostic Complete. If all positions were reached, your mapping is 100% safe.")

if __name__ == "__main__":
    run_diagnostic()