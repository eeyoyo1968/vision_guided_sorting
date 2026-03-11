
# Vision-Guided Robotic Sorting System

## Operating Instructions

### 1. Overview
This document describes the hardware setup, software configuration, and operational workflow for the Vision‑Guided Robotic Sorting System using **ROS 2 Humble**, a **UR12e robot**, **Intel RealSense D432 depth camera**, **MoveIt2**, and a **Robotiq 2F‑85 gripper**.

---

## 2. Hardware Setup
### 2.1 Connections
- Host PC (IP: `192.168.2.1`) → UR Controller (IP: `192.168.2.2`) via Ethernet
- Intel RealSense D432 camera → Host PC via USB

### 2.2 ROS2 Workspace Structure
```
~/humble_ws/
~/humble_ws/src/my_ur_description/
~/humble_ws/src/my_ur_description/scripts/
```

---

## 3. UR Controller (Polyscope X) Configuration
### Steps
1. Power on the UR controller
2. Power on and unlock the robot

### 3.1 Network Configuration
Navigate: **Settings → Network**
- Mode: **Static**
- IP: `192.168.2.2`
- Subnet Mask: `255.255.255.0`

### 3.2 Services Configuration
Navigate: **Settings → Services**
Ensure the following are enabled:
- Primary Client Interface
- Secondary Client Interface
- Real-Time Client Interface
- RTDE
- Interpreter Mode Socket
- Modbus TCP Server
- ROS2

### 3.3 URCaps
Navigate: **Application**
- External Control URCap → Host `192.168.2.1`, Port `50002`
- Tool Communication Forwarder → Ensure **socat server** is running

### 3.4 Program Setup
Ensure program includes:
```
set_tool_voltage(24)
set_tool_communication(True, 115200, 0, 1, 1.5, 3.5)
```
Ensure **External Control** program from PC is cached and selectable.

---

## 4. PC Setup
### 4.1 Terminals
Open multiple terminals (up to **9**).

### 4.2 .bashrc
Ensure the following exist in **every** terminal:
```
source /opt/ros/humble/setup.bash
source ~/humble_ws/install/setup.bash
export ROS_DOMAIN_ID=43
```

---

## 5. Terminal-by-Terminal Launch
### Terminal 1 — Perception
```
ros2 run grasp_perception grasp_node5
```

### Terminal 2 — Camera Parameter Control
(Launch RealSense parameter node.)

### Terminal 3 — UR Driver
```
ros2 launch ur_robot_driver ur_control.launch.py   ur_type:=ur12e   robot_ip:=192.168.2.2   use_fake_hardware:=false   kinematics_params:=./src/my_ur_description/config/ur12e_calibration.yaml   initial_joint_controller:=scaled_joint_trajectory_controller
```

### Terminal 4 — Controllers + MoveIt2
Check controllers:
```
ros2 control list_controllers
```
Activate controller if needed:
```
ros2 control switch_controllers --activate scaled_joint_trajectory_controller --deactivate joint_trajectory_controller
```
Launch MoveIt2:
```
ros2 launch ur_moveit_config ur_moveit.launch.py   ur_type:=ur12e   launch_rviz:=true
```

### Terminal 5 — Scene Setup & Test Motions
```
cd ~/humble_ws/src/my_ur_description/scripts
python3 setup_moveit_scene2.py
python3 move_plan_xyz_theta.py
python3 home.py
```

### Terminal 6 — SOCAT Gripper Bridge
```
while true; do
  echo 'Reviving Bridge...'
  sudo killall -9 socat 2>/dev/null
  socat pty,link=/tmp/ttyUR,raw,echo=0,waitslave tcp:192.168.2.2:54321,nodelay
  sleep 1
done
```

### Terminal 7 — Gripper Link & Test
```
ls -l /tmp/ttyUR
sudo chmod 666 /tmp/ttyUR
python3 test_gripper_final_fixed_v3.py
python3 careful_gripper_test.py
python3 force_release2.py
```

### Terminal 8 — Gripper Server Node
```
cd ~/humble_ws/src/my_ur_description/scripts
python3 robotiq_gripper_server.py
# or
python3 robotiq_gripper_server2.py
```

### Terminal 9 — Brain Logic
Single pick:
```
python3 vision_guided_sorting_brain_once_gripper_client.py
```
Force-control pick:
```
python3 vision_guided_sorting_brain_once_gripper_client_force_contact.py
```
Repeated sorting:
```
./run_picker_position.sh
./run_picker_force.sh
```
To stop:
```
CTRL-C
```
Return home:
```
python3 home.py
```

---

## Quick‑Start Guide
### Step 1 — Hardware
- Connect PC ↔ UR Controller
- Connect Intel D432 camera

### Step 2 — UR Setup
- Set IP to `192.168.2.2`
- Configure External Control URCap
- Run External Control program

### Step 3 — PC Setup
```
source /opt/ros/humble/setup.bash
source ~/humble_ws/install/setup.bash
export ROS_DOMAIN_ID=43
```

### Step 4 — Launch Sequence
1. Perception node
2. Camera node
3. UR driver
4. MoveIt2
5. Scene & motion testing
6. SOCAT gripper bridge
7. Gripper test
8. Gripper server
9. Sorting logic

### Step 5 — Run Tasks
- Single pick
- Repeated cycle
- Force or position control

### Step 6 — Shutdown
```
CTRL-C
python3 home.py
```
