# Complete Refactoring Guide for Your UR12e Setup

## Current Situation Analysis

**What you have:**
- ✅ Custom `my_ur_description` package with `ur_system.xacro` and `ur_srdf.xacro`
- ✅ Using `robotiq_description` package for gripper (good!)
- ✅ Proper scene modeling (desk, walls, cameras)
- ✅ Working fake hardware and URSim integration
- ✅ Proper controller configurations

**What you modified in UR packages:**
1. **ur_description/urdf/inc/ur_joint_control.xacro** - Removed velocity command interfaces
2. **ur_description/urdf/ur.urdf.xacro** - Changed defaults, added ros2_control call
3. **ur_robot_driver/config/ur_controllers.yaml** - Added gripper controller
4. **ur_robot_driver/hardware_interface_plugin.xml** - Added duplicate entry (likely unnecessary)

**Good news:** You already have velocity command interfaces in your custom `ur_system.xacro` (lines 152, 159, 166, 173, 180, 187), so the modification to remove them was UNNECESSARY!

## Strategy: Minimal Changes Required

You actually need very few changes because your `ur_system.xacro` already defines its own complete ros2_control block. The key is to ensure the UR packages are pristine and your custom package is self-contained.

---

## Step-by-Step Refactoring Plan

### STEP 1: Revert Universal Robots Packages

```bash
cd ~/your_workspace/src/Universal_Robots_ROS2_Description
git checkout .
git clean -fd

cd ~/your_workspace/src/Universal_Robots_ROS2_Driver
git checkout .
git clean -fd
```

### STEP 2: Move Controller Config to Your Package

The gripper controller config needs to be in YOUR package's controller file.

**Current situation:**
- You have `ur_controllers.yaml` which has the gripper controller ✅
- This is ALREADY in your package, not the UR package

**Action Required:** None! Your `ur_controllers.yaml` is already correct.

### STEP 3: Verify Your Package Structure

Your `my_ur_description` package should have:

```
my_ur_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── ur_system.xacro          ✅ You have this
│   └── ur_system.srdf           ✅ You have this (or ur_srdf.xacro)
├── srdf/
│   └── ur_system.srdf           ✅ Create from ur_srdf.xacro
├── config/
│   ├── ur_controllers.yaml      ✅ You have this
│   ├── ros2_controllers.yaml    ✅ You have this (for fake hardware)
│   ├── moveit_controllers.yaml  ✅ You have this
│   ├── kinematics.yaml          ✅ You have this
│   └── joint_limits.yaml        ✅ You have this
├── launch/
│   └── my_robot.launch.py       ✅ You have this
├── rviz/
│   └── my_robot_view.rviz       ✅ You have this
└── scripts/
    └── test_move_xyz_theta_noflip_gmove.py  ✅ You have this
```

### STEP 4: Fix the SRDF File

Your `ur_srdf.xacro` needs to be converted to a proper SRDF file for the launch file.

**Option A: Generate SRDF from xacro (Recommended)**

Update your launch file to generate SRDF from xacro:

```python
# In my_robot.launch.py, replace this:
srdf_file = os.path.join(get_package_share_directory(description_pkg), 'srdf', 'ur_system.srdf')
with open(srdf_file, 'r') as f:
    semantic_config = f.read()

# With this:
srdf_xacro_file = PathJoinSubstitution([
    get_package_share_directory(description_pkg), 
    'urdf', 
    'ur_srdf.xacro'
])

robot_description_semantic_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ", srdf_xacro_file,
])
robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}
```

**Option B: Pre-generate SRDF**

```bash
cd ~/your_workspace/src/my_ur_description
mkdir -p srdf
ros2 run xacro xacro urdf/ur_srdf.xacro > srdf/ur_system.srdf
```

### STEP 5: Update CMakeLists.txt

Make sure your `CMakeLists.txt` installs all necessary files:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_ur_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install all directories
install(DIRECTORY
  urdf
  srdf
  config
  launch
  rviz
  meshes  # if you have any
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/test_move_xyz_theta_noflip_gmove.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

### STEP 6: Update package.xml Dependencies

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_ur_description</name>
  <version>1.0.0</version>
  <description>UR12e workcell with Robotiq 2F-85 gripper</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- UR Dependencies -->
  <exec_depend>ur_description</exec_depend>
  <exec_depend>ur_robot_driver</exec_depend>
  <exec_depend>ur_client_library</exec_depend>
  
  <!-- Robotiq Dependencies -->
  <exec_depend>robotiq_description</exec_depend>
  
  <!-- RealSense Dependencies (for camera visualization) -->
  <exec_depend>realsense2_description</exec_depend>
  
  <!-- ROS2 Control -->
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>joint_trajectory_controller</exec_depend>
  <exec_depend>ur_controllers</exec_depend>
  <exec_depend>mock_components</exec_depend>
  
  <!-- MoveIt -->
  <exec_depend>moveit_ros_move_group</exec_depend>
  <exec_depend>moveit_kinematics</exec_depend>
  <exec_depend>moveit_planners_ompl</exec_depend>
  <exec_depend>moveit_simple_controller_manager</exec_depend>
  
  <!-- Tools -->
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### STEP 7: Update Your ur_system.xacro (Minor Fix)

Your current `ur_system.xacro` has the gripper set to ALWAYS use fake hardware (lines 200-202):

```xml
<xacro:unless value="$(arg use_fake_hardware)">
   <plugin>mock_components/GenericSystem</plugin>  <!-- This should be real gripper driver -->
</xacro:unless>
```

**If you want real gripper support later**, change this section to:

```xml
<ros2_control name="RobotiqGripperHardwareInterface" type="system">
  <hardware>
    <!-- Always use fake/mock for gripper since we don't have the real hardware interface -->
    <plugin>mock_components/GenericSystem</plugin>
    <param name="calculate_dynamics">false</param>
  </hardware>
  
  <joint name="robotiq_85_left_knuckle_joint">
    <command_interface name="position" />
    <state_interface name="position">
      <param name="initial_value">0.0</param>  <!-- 0.0 = open, 0.8 = closed -->
    </state_interface>
    <state_interface name="velocity"/>
  </joint>
  
  <!-- All mimic joints below -->
  <joint name="robotiq_85_right_knuckle_joint">
    <param name="mimic">robotiq_85_left_knuckle_joint</param>
    <param name="multiplier">-1.0</param>
    <state_interface name="position"/>
  </joint>
  <!-- ... rest of mimic joints ... -->
</ros2_control>
```

**Note:** The initial value of 0.795 in your current file would start the gripper nearly closed. Change to 0.0 for open.

### STEP 8: Controller Configuration Files - ALREADY CORRECT

Your `ur_controllers.yaml` is perfect and already includes:
- `scaled_joint_trajectory_controller` for real/URSim hardware
- `joint_trajectory_controller` for fake hardware  
- `robotiq_gripper_controller` for the gripper

**No changes needed!**

### STEP 9: Launch File - One Small Fix

Your launch file is excellent! Just one recommendation:

Change line ~37 to properly use the LaunchConfiguration:

```python
# Current (line ~37):
trajectory_controller_name = "scaled_joint_trajectory_controller"
if use_fake_hardware_config == "true":  # This comparison won't work as expected
    trajectory_controller_name = "joint_trajectory_controller"

# Better approach:
# Define both possible controller configurations
moveit_controllers_fake = {
    'moveit_simple_controller_manager': {
        'controller_names': ['joint_trajectory_controller', 'robotiq_gripper_controller'],
        'joint_trajectory_controller': {
            'type': 'FollowJointTrajectory',
            'action_ns': 'follow_joint_trajectory',
            'default': True,
            'joints': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                      'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
        },
        'robotiq_gripper_controller': {
            'type': 'FollowJointTrajectory',
            'action_ns': 'follow_joint_trajectory',
            'joints': ['robotiq_85_left_knuckle_joint'],
        },
    }
}

moveit_controllers_real = {
    'moveit_simple_controller_manager': {
        'controller_names': ['scaled_joint_trajectory_controller', 'robotiq_gripper_controller'],
        'scaled_joint_trajectory_controller': {
            'type': 'FollowJointTrajectory',
            'action_ns': 'follow_joint_trajectory',
            'default': True,
            'joints': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                      'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
        },
        'robotiq_gripper_controller': {
            'type': 'FollowJointTrajectory',
            'action_ns': 'follow_joint_trajectory',
            'joints': ['robotiq_85_left_knuckle_joint'],
        },
    }
}

# Then in move_group Node, use OpaqueFunction to select the right one
# Or use two separate move_group nodes with IfCondition/UnlessCondition
```

However, your current approach will work for most cases. This is just a refinement.

---

## STEP 10: Testing After Refactoring

### A. Build the workspace

```bash
cd ~/your_workspace
colcon build --packages-select my_ur_description
source install/setup.bash
```

### B. Test URDF generation

```bash
# Test with fake hardware
ros2 run xacro xacro $(ros2 pkg prefix my_ur_description)/share/my_ur_description/urdf/ur_system.xacro \
  use_fake_hardware:=true > /tmp/robot_fake.urdf

# Test with real hardware
ros2 run xacro xacro $(ros2 pkg prefix my_ur_description)/share/my_ur_description/urdf/ur_system.xacro \
  use_fake_hardware:=false robot_ip:=192.168.56.101 > /tmp/robot_real.urdf

# Validate
check_urdf /tmp/robot_fake.urdf
check_urdf /tmp/robot_real.urdf
```

### C. Launch with fake hardware

```bash
ros2 launch my_ur_description my_robot.launch.py use_fake_hardware:=true
```

### D. Launch with URSim

```bash
# First, start URSim container
docker-compose up -d simulator

# Wait for URSim to start, then launch
ros2 launch my_ur_description my_robot.launch.py \
  use_fake_hardware:=false \
  robot_ip:=127.0.0.1
```

### E. Test gripper control

```bash
# Open gripper
ros2 action send_goal /robotiq_gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['robotiq_85_left_knuckle_joint'],
      points: [
        { positions: [0.0], time_from_start: { sec: 2 } }
      ]
    }
  }"

# Close gripper
ros2 action send_goal /robotiq_gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['robotiq_85_left_knuckle_joint'],
      points: [
        { positions: [0.8], time_from_start: { sec: 2 } }
      ]
    }
  }"
```

---

## Summary: What You Actually Need to Change

### Files to Revert (in UR packages):
1. ✅ `Universal_Robots_ROS2_Description/urdf/inc/ur_joint_control.xacro`
2. ✅ `Universal_Robots_ROS2_Description/urdf/ur.urdf.xacro`
3. ✅ `Universal_Robots_ROS2_Driver/config/ur_controllers.yaml`
4. ✅ `Universal_Robots_ROS2_Driver/hardware_interface_plugin.xml`

### Files to Keep/Modify (in YOUR package):
1. ✅ `ur_system.xacro` - Keep as is (maybe adjust gripper initial position)
2. ✅ `ur_srdf.xacro` - Keep as is
3. ✅ `ur_controllers.yaml` - Keep as is
4. ✅ `ros2_controllers.yaml` - Keep as is
5. ✅ `my_robot.launch.py` - Minor improvements optional
6. ✅ `package.xml` - Add dependencies listed above
7. ✅ `CMakeLists.txt` - Update as shown above

### What This Achieves:
- ✅ Original UR packages remain untouched
- ✅ All customizations in one place
- ✅ Easy to update UR packages
- ✅ Clear, maintainable structure
- ✅ Works with both fake hardware and URSim
- ✅ Gripper properly integrated

---

## Why Your Original Modifications Were Unnecessary

1. **Removing velocity command interfaces** - Your `ur_system.xacro` defines its own complete ros2_control block (lines 123-192), so changes to UR's `ur_joint_control.xacro` don't affect you at all!

2. **Modifying `ur.urdf.xacro`** - You're not using this file. You use `ur_macro.xacro` directly in your `ur_system.xacro` (line 7).

3. **Adding gripper to UR's controller yaml** - Should be in YOUR controller yaml (which you already have).

4. **Plugin.xml modification** - Likely unnecessary duplication.

Your system works DESPITE these modifications, not because of them. Your custom files already handle everything properly!
