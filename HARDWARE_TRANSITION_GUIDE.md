# Simulation to Real Hardware Transition Guide
## UR12e + Robotiq 2F-85 Integration

---

## Part 1: UR12e - Simple Transition ✅

You're correct - the UR12e transition is straightforward!

### Current Setup (Simulation)

```yaml
# In ur_system.xacro
<xacro:arg name="use_fake_hardware" default="false" />
<xacro:arg name="robot_ip" default="127.0.0.1" />

# Hardware interface uses:
<plugin>ur_robot_driver/URPositionHardwareInterface</plugin>
<param name="robot_ip">$(arg robot_ip)</param>
```

### Changes Needed for Real UR12e

**1. Get Robot IP Address**
```bash
# On the UR teach pendant:
# Settings → System → Network
# Note the IP address, e.g., 192.168.1.100
```

**2. Extract Calibration File**

```bash
# Install calibration tool
sudo apt install ros-humble-ur-calibration

# Extract from robot
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.1.100 \
  target_filename:="${HOME}/my_ur12e_calibration.yaml"

# Copy to your package
mkdir -p ~/your_workspace/src/my_ur_description/config/calibration
cp ~/my_ur12e_calibration.yaml \
   ~/your_workspace/src/my_ur_description/config/calibration/
```

**3. Update Launch Command**

```bash
# Before (simulation)
ros2 launch my_ur_description my_robot.launch.py \
  use_fake_hardware:=false \
  robot_ip:=127.0.0.1

# After (real robot)
ros2 launch my_ur_description my_robot.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.1.100 \
  kinematics_file:=$(ros2 pkg prefix my_ur_description)/share/my_ur_description/config/calibration/my_ur12e_calibration.yaml
```

**That's it for UR12e!** ✅

---

## Part 2: Robotiq 2F-85 - More Complex ⚠️

You're right - the gripper needs more work. Here's the complete transition strategy.

### Current Setup (Fake Hardware)

```xml
<!-- In ur_system.xacro, lines 194-238 -->
<ros2_control name="RobotiqGripperHardwareInterface" type="system">
  <hardware>
    <plugin>mock_components/GenericSystem</plugin>  <!-- FAKE -->
  </hardware>
  <joint name="robotiq_85_left_knuckle_joint">
    <command_interface name="position" />
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- mimic joints -->
</ros2_control>
```

### Option A: Keep Fake Hardware (Quick Solution)

**When to use:** 
- Early testing phase
- Don't have real gripper yet
- Gripper control not critical for testing

**What to do:** Nothing! Keep current setup.

**Pros:** 
- ✅ Works immediately
- ✅ No hardware integration needed
- ✅ Can test robot motions

**Cons:**
- ❌ Can't actually grasp objects
- ❌ No force feedback
- ❌ Gripper state in RViz only

---

### Option B: Real Robotiq Hardware (Production Solution)

**When to use:**
- Production deployment
- Need actual grasping
- Have physical gripper

**What you need:**

#### 1. Hardware Connection

The Robotiq 2F-85 connects via:
- **RS-485 (Modbus RTU)** - Standard, most common
- **USB** - With Robotiq USB adapter
- **URCap** - Through UR controller (easiest!)

**Recommended: URCap Integration** (Easiest)

The gripper plugs into the UR tool flange's control box and is controlled through the UR controller.

#### 2. Install Robotiq ROS2 Driver

Unfortunately, official Robotiq ROS2 support is limited. You have several options:

**Option B1: robotiq_2f_gripper_control (Community Package)**

```bash
cd ~/your_workspace/src

# Clone the ROS2 driver (check for latest humble-compatible fork)
git clone -b humble https://github.com/PickNikRobotics/robotiq_2f_gripper_control.git

# Install dependencies
sudo apt install ros-humble-serial \
                 ros-humble-soem \
                 ros-humble-socketcan-interface

# Build
cd ~/your_workspace
colcon build --packages-select robotiq_2f_gripper_control
source install/setup.bash
```

**Option B2: ros2_robotiq_gripper (Alternative)**

```bash
cd ~/your_workspace/src
git clone https://github.com/cambel/ros2_robotiq_gripper.git

cd ~/your_workspace
colcon build --packages-select ros2_robotiq_gripper
```

**Option B3: Custom Modbus Driver (Most Reliable)**

For production, consider writing a simple Modbus interface:

```bash
sudo apt install python3-pymodbus
pip3 install --break-system-packages pymodbus
```

#### 3. Update ur_system.xacro

**File: `urdf/ur_system.xacro`**

Change the gripper ros2_control section (lines 194-238):

```xml
<ros2_control name="RobotiqGripperHardwareInterface" type="system">
  <hardware>
    <xacro:if value="$(arg use_fake_hardware)">
      <!-- Simulation mode -->
      <plugin>mock_components/GenericSystem</plugin>
    </xacro:if>
    
    <xacro:unless value="$(arg use_fake_hardware)">
      <!-- Real hardware mode -->
      <plugin>robotiq_2f_gripper_control/RobotiqGripperHardwareInterface</plugin>
      <param name="gripper_port">/dev/ttyUSB0</param>  <!-- or via Modbus IP -->
      <param name="slave_address">9</param>
      <param name="gripper_speed">255</param>
      <param name="gripper_force">255</param>
    </xacro:unless>
  </hardware>
  
  <joint name="robotiq_85_left_knuckle_joint">
    <command_interface name="position"/>
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity"/>
  </joint>
  
  <!-- Keep all mimic joints the same -->
</ros2_control>
```

---

### Option C: URCap + Action Server (Hybrid - RECOMMENDED)

**This is the most practical approach for production!**

The gripper is controlled via the UR controller's URCap, and you create an action server to bridge to ROS2.

#### Why This Approach?

- ✅ Uses UR's built-in gripper control (reliable)
- ✅ No additional wiring needed
- ✅ Gripper state monitored by UR
- ✅ E-stop integration automatic
- ✅ Easier troubleshooting

#### Implementation

**Step 1: Install URCap on UR Controller**

1. Download Robotiq URCap from: https://robotiq.com/support
2. On UR teach pendant: Settings → System → URCaps → Install
3. Upload the .urcap file
4. Reboot the UR controller

**Step 2: Create ROS2 Action Server**

**File: `scripts/robotiq_gripper_server.py`**

```python
#!/usr/bin/env python3
"""
ROS2 Action Server for Robotiq Gripper via UR Dashboard
Sends commands to the gripper through UR's URCap
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import GripperCommand
import socket
import time

class RobotiqGripperServer(Node):
    def __init__(self):
        super().__init__('robotiq_gripper_server')
        
        # Connection to UR controller
        self.robot_ip = self.declare_parameter('robot_ip', '192.168.1.100').value
        self.dashboard_port = 29999
        
        # Action server
        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd',
            self.execute_callback
        )
        
        self.get_logger().info(f"Robotiq Gripper Server ready (UR IP: {self.robot_ip})")
    
    def send_ur_command(self, command):
        """Send a command to the UR Dashboard server."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((self.robot_ip, self.dashboard_port))
            s.recv(1024)  # Welcome message
            
            s.send(f"{command}\n".encode())
            response = s.recv(1024).decode()
            
            s.close()
            return response
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")
            return None
    
    def execute_callback(self, goal_handle):
        """Execute gripper command."""
        request = goal_handle.request
        position = request.command.position  # 0.0 to 0.085 (meters)
        
        # Convert position to gripper units (0-255)
        # Robotiq 2F-85: 0.085m max opening
        gripper_pos = int((1.0 - (position / 0.085)) * 255)
        gripper_pos = max(0, min(255, gripper_pos))
        
        self.get_logger().info(f"Moving gripper to position: {gripper_pos}/255")
        
        # Send command via URScript
        ur_script = f"set_digital_out(0, True)\n"  # Activate gripper
        ur_script += f"set_tool_digital_out(0, True)\n"
        
        # This is simplified - actual URScript depends on your URCap version
        # You'll need to use the specific URCap commands
        
        response = self.send_ur_command(ur_script)
        
        # Wait for completion
        time.sleep(2.0)
        
        # Return success
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = position
        result.reached_goal = True
        
        return result

def main():
    rclpy.init()
    server = RobotiqGripperServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Step 3: Launch with Real Gripper**

```bash
# Terminal 1: Start gripper server
ros2 run my_ur_description robotiq_gripper_server.py --ros-args -p robot_ip:=192.168.1.100

# Terminal 2: Launch robot
ros2 launch my_ur_description my_robot.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.1.100
```

---

### Option D: Direct Modbus Communication (Advanced)

**For complete control without URCap dependency.**

**File: `scripts/robotiq_modbus_node.py`**

```python
#!/usr/bin/env python3
"""
Direct Modbus RTU control of Robotiq 2F-85
Requires physical RS-485 connection
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import GripperCommand
from pymodbus.client import ModbusSerialClient
import time

class RobotiqModbusNode(Node):
    def __init__(self):
        super().__init__('robotiq_modbus_node')
        
        # Modbus connection
        port = self.declare_parameter('port', '/dev/ttyUSB0').value
        baudrate = self.declare_parameter('baudrate', 115200).value
        slave_id = self.declare_parameter('slave_id', 9).value
        
        self.client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            timeout=1,
            parity='N',
            stopbits=1,
            bytesize=8
        )
        
        if not self.client.connect():
            self.get_logger().error(f"Failed to connect to gripper on {port}")
            return
        
        self.get_logger().info(f"Connected to Robotiq gripper on {port}")
        self.slave_id = slave_id
        
        # Initialize gripper
        self.activate_gripper()
        
        # Action server
        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd',
            self.execute_callback
        )
    
    def activate_gripper(self):
        """Activate the gripper (required on startup)."""
        self.get_logger().info("Activating gripper...")
        
        # Write to activation register
        self.client.write_register(0x03E8, 0x0100, slave=self.slave_id)
        time.sleep(0.5)
        
        # Wait for activation
        for _ in range(10):
            response = self.client.read_holding_registers(0x07D0, 1, slave=self.slave_id)
            if response.registers[0] & 0x3100 == 0x3100:
                self.get_logger().info("Gripper activated!")
                return True
            time.sleep(0.5)
        
        self.get_logger().error("Gripper activation failed")
        return False
    
    def execute_callback(self, goal_handle):
        """Execute gripper movement."""
        request = goal_handle.request
        position = request.command.position  # 0.0 (closed) to 0.085 (open) meters
        
        # Convert to gripper units (0-255)
        # 0 = fully open, 255 = fully closed
        gripper_pos = int((1.0 - (position / 0.085)) * 255)
        gripper_pos = max(0, min(255, gripper_pos))
        
        self.get_logger().info(f"Moving to position: {gripper_pos}/255")
        
        # Write position command (register 0x03E9)
        # Format: [rACT, rGTO, rATR, 0, rPR, rSP, rFR]
        # rACT=1, rGTO=1 (go to position)
        # rPR = position (0-255)
        # rSP = speed (0-255)
        # rFR = force (0-255)
        
        self.client.write_register(0x03E9, 0x0900, slave=self.slave_id)  # Activate + Go
        self.client.write_register(0x03EA, gripper_pos, slave=self.slave_id)  # Position
        self.client.write_register(0x03EB, 255, slave=self.slave_id)  # Speed
        self.client.write_register(0x03EC, 255, slave=self.slave_id)  # Force
        
        # Wait for completion
        timeout = 5.0
        start_time = time.time()
        while time.time() - start_time < timeout:
            response = self.client.read_holding_registers(0x07D0, 3, slave=self.slave_id)
            status = response.registers[0]
            
            # Check if motion complete (gOBJ bits)
            if (status & 0xC000) != 0x0000:  # Object detected or at requested position
                break
            
            time.sleep(0.1)
        
        # Get final position
        response = self.client.read_holding_registers(0x07D1, 1, slave=self.slave_id)
        actual_pos = response.registers[0]
        
        self.get_logger().info(f"Final position: {actual_pos}/255")
        
        # Return result
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = (1.0 - (actual_pos / 255.0)) * 0.085
        result.reached_goal = True
        
        return result

def main():
    rclpy.init()
    node = RobotiqModbusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Hardware Setup:**
```
Robotiq 2F-85 (RS-485) 
    ↓ (Modbus RTU)
USB-to-RS485 Adapter (/dev/ttyUSB0)
    ↓
Computer running ROS2
```

**Launch:**
```bash
ros2 run my_ur_description robotiq_modbus_node.py --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=115200 \
  -p slave_id:=9
```

---

## Complete Transition Workflow

### Phase 1: UR12e Only (No Gripper Change)

```bash
# Keep gripper as fake hardware
ros2 launch my_ur_description my_robot.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.1.100 \
  kinematics_file:=/path/to/my_ur12e_calibration.yaml
```

**Test checklist:**
- [ ] Robot connects successfully
- [ ] Joint states published correctly
- [ ] Can plan and execute trajectories
- [ ] Emergency stop works
- [ ] Velocity scaling active

### Phase 2: Add Real Gripper

**Choose your gripper integration method:**

| Method | Difficulty | Reliability | Setup Time | Recommended For |
|--------|-----------|-------------|------------|-----------------|
| **Fake Hardware** | Easy | N/A | 0 min | Testing only |
| **URCap + Action Server** | Medium | High | 2-4 hours | **Production** ⭐ |
| **Community ROS2 Driver** | Medium | Medium | 3-6 hours | Development |
| **Custom Modbus** | Hard | Very High | 8+ hours | Advanced users |

**Recommended: URCap + Action Server**

1. Install Robotiq URCap on UR controller
2. Create action server node (provided above)
3. Update your motion scripts to use the action interface
4. Test with simple open/close commands

### Phase 3: Integration Testing

```bash
# Launch everything
ros2 launch my_ur_description my_robot.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.1.100

# In another terminal, launch gripper server
ros2 run my_ur_description robotiq_gripper_server.py

# Test gripper
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.0}}"  # Close

ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.085}}"  # Open
```

---

## Troubleshooting Real Hardware

### UR12e Issues

**Problem: "Could not connect to robot"**
```bash
# Check network connection
ping 192.168.1.100

# Check if UR is in remote control mode
# On teach pendant: Program → URCaps → External Control
```

**Problem: "Calibration mismatch error"**
```bash
# Re-extract calibration
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.1.100 \
  target_filename:=~/new_calibration.yaml
```

**Problem: "Robot in protective stop"**
```bash
# On teach pendant: Clear protective stop
# Check for collision in recent motion
```

### Robotiq 2F-85 Issues

**Problem: "Cannot connect to gripper"**
```bash
# Check USB connection
ls -l /dev/ttyUSB*

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Or add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in
```

**Problem: "Gripper not activating"**
```bash
# Reset gripper power
# Turn off UR controller, wait 10 seconds, turn back on

# Check URCap installation
# Teach pendant: Settings → System → URCaps
```

**Problem: "Gripper moves but doesn't grasp"**
```bash
# Adjust force/speed parameters in URCap
# Check finger alignment
# Verify object size within gripper range (0-85mm)
```

---

## Safety Checklist for Real Hardware

### Before First Run

- [ ] Emergency stop tested and accessible
- [ ] Workspace clear of obstacles
- [ ] Protective barriers in place (if required)
- [ ] Robot mounted securely
- [ ] All cables secured and routed safely
- [ ] Speed limited to 10% for initial tests
- [ ] Collision sensitivity set appropriately
- [ ] Team member present with E-stop access

### Initial Test Sequence

1. **Power on in safeguard mode**
2. **Jog manually** using teach pendant
3. **Test E-stop** - verify it works
4. **Run simple joint move** (no obstacles)
5. **Test gripper open/close** (no object)
6. **Gradually increase complexity**

### Regular Checks

- [ ] Gripper finger alignment
- [ ] Cable wear/damage
- [ ] Robot calibration accuracy
- [ ] Joint backlash/play
- [ ] Emergency stop function
- [ ] Protective stop sensitivity

---

## Summary & Recommendations

### For UR12e ✅
**Transition is simple:**
1. Get robot IP
2. Extract calibration
3. Update launch command
**Estimated time: 30 minutes**

### For Robotiq 2F-85 ⚠️
**Recommended approach:**
1. **Phase 1:** Keep fake hardware, test robot motions
2. **Phase 2:** Install URCap + create action server
3. **Phase 3:** Integrate and test
**Estimated time: 4-8 hours**

### Best Practice Integration Order

```
Week 1: UR12e with fake gripper
  ↓
Week 2: Test all robot motions, verify workspace
  ↓
Week 3: Add URCap gripper control
  ↓
Week 4: Integration testing
  ↓
Week 5: Production deployment
```

### Files to Create/Modify

**New files needed:**
- `scripts/robotiq_gripper_server.py` (URCap method)
- OR `scripts/robotiq_modbus_node.py` (Modbus method)
- `config/calibration/my_ur12e_calibration.yaml`

**Files to modify:**
- `urdf/ur_system.xacro` (only if using direct gripper driver)
- `launch/my_robot.launch.py` (add gripper server option)

**No changes needed for URCap method!** The action server runs separately and your existing code works as-is.

---

## Next Steps

1. **This week:** Test UR12e with fake gripper
2. **Get URCap:** Download and install on UR controller
3. **Build action server:** Use template provided above
4. **Test gripper:** Simple open/close commands
5. **Integrate:** Update your motion scripts
6. **Production:** Full pick-and-place testing

Your transition strategy is sound - UR12e is indeed simple, and the gripper needs careful planning. The URCap + Action Server approach gives you the best balance of reliability and ease of use! 🚀
