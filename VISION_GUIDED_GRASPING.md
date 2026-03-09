# Vision-Guided Grasping Integration Guide
## Intel RealSense D435 + YOLOv8 + UR12e + Robotiq 2F-85

---

## System Architecture Overview

```
┌─────────────────┐      ┌──────────────────┐      ┌─────────────────┐
│ Perception Node │─────>│ Grasp Planner    │─────>│ Motion Control  │
│ (Your Colleague)│      │ (You)            │      │ (UR12e + Grip)  │
└─────────────────┘      └──────────────────┘      └─────────────────┘
     │                           │                         │
     │ Publishes:                │ Transforms:             │ Executes:
     │ - Object class            │ - Camera → Base         │ - Pick motion
     │ - 3D position             │ - Add grasp offset      │ - Place motion
     │ - Bounding box            │ - Choose bin            │ - Avoid collision
     │ - Confidence              │                         │
     └───────────────────────────┴─────────────────────────┘
                                 TF2 Transform Tree
```

---

## Step 1: Define the Interface Between Perception and Manipulation

### Option A: Custom Message (Recommended)

Create a custom message for detected objects.

**File: `msg/DetectedObject.msg`**
```
# Header with timestamp and frame_id
std_msgs/Header header

# Object information
string object_class           # e.g., "bottle", "box", "screwdriver"
float32 confidence           # Detection confidence (0.0 to 1.0)

# 3D position in camera frame
geometry_msgs/Point position  # x, y, z in meters

# Optional: Orientation (if your colleague can estimate it)
geometry_msgs/Quaternion orientation

# Optional: Bounding box for visualization
float32 width
float32 height
float32 depth

# Optional: Grasp quality score (if computed)
float32 grasp_quality
```

**File: `msg/DetectedObjectArray.msg`**
```
std_msgs/Header header
DetectedObject[] objects
```

### Option B: Use Standard Messages (Quick Start)

Use `geometry_msgs/PoseStamped` or `vision_msgs/Detection3D`:

```bash
# Install vision_msgs
sudo apt install ros-humble-vision-msgs
```

---

## Step 2: TF2 Transform Setup

### Your Existing TF Tree

```
world
 └─ easydesk
     ├─ base_link (robot base)
     │   └─ ... (robot links)
     │       └─ tool0 (end-effector)
     │           └─ robotiq_85_base_link (gripper)
     ├─ cam_left_camera_link
     │   └─ cam_left_camera_optical_frame
     └─ cam_right_camera_link
         └─ cam_right_camera_optical_frame
```

### What Your Colleague Needs to Publish

Their perception node should publish transforms for:

1. **Camera to Object** (for each detected object)
2. **Camera Frame** (already in your URDF at fixed position)

**Example perception node structure:**

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        
    def publish_detected_object(self, obj_class, x_cam, y_cam, z_cam):
        """
        Publish detected object transform.
        x_cam, y_cam, z_cam are in camera optical frame (meters)
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'cam_left_camera_optical_frame'  # Or whichever camera
        t.child_frame_id = f'object_{obj_class}_{int(time.time())}'
        
        # Position from camera
        t.transform.translation.x = float(x_cam)
        t.transform.translation.y = float(y_cam)
        t.transform.translation.z = float(z_cam)
        
        # Identity rotation (or estimated orientation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
```

---

## Step 3: Coordinate Frame Conventions

### RealSense D435 Optical Frame

```
Camera Optical Frame:
  X: Right (when looking from behind camera)
  Y: Down
  Z: Forward (into the scene)

Robot Base Frame:
  X: Forward
  Y: Left
  Z: Up
```

**Important:** Your URDF already handles this transformation!  
Check lines 94-99 in `ur_system.xacro`:

```xml
<link name="cam_left_camera_optical_frame"/>
<joint name="cam_left_optical_joint" type="fixed">
  <parent link="cam_left_camera_link"/>
  <child link="cam_left_camera_optical_frame"/>
  <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
</joint>
```

This rotates from camera convention to ROS convention. ✅

---

## Step 4: Create Vision-Guided Grasp Node

### File: `scripts/vision_guided_grasp.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import math

# Import your existing controller
from test_move_xyz_theta_noflip_gmove import UR12eController

class VisionGuidedGrasp(Node):
    def __init__(self):
        super().__init__('vision_guided_grasp')
        
        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Robot controller
        self.robot = UR12eController()
        
        # Subscribe to detected objects
        self.create_subscription(
            PoseStamped,
            '/detected_object_pose',  # Topic from perception
            self.object_callback,
            10
        )
        
        # Grasp parameters
        self.approach_height = 0.15  # 15cm above object
        self.grasp_offset_z = -0.10  # 10cm below camera detection point
        
        # Bin locations (in base_link frame)
        self.bins = {
            'bottle': (0.5, 0.3, 0.4),
            'box': (0.5, -0.3, 0.4),
            'default': (0.6, 0.0, 0.4)
        }
        
        self.get_logger().info("Vision-guided grasp node ready!")
    
    def object_callback(self, msg):
        """Called when perception detects an object."""
        self.get_logger().info(f"Detected object in frame: {msg.header.frame_id}")
        
        # Transform object pose to base_link
        try:
            # Wait for transform (timeout 2 seconds)
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Transform the pose
            object_in_base = do_transform_pose(msg.pose, transform)
            
            self.get_logger().info(
                f"Object position in base_link: "
                f"x={object_in_base.position.x:.3f}, "
                f"y={object_in_base.position.y:.3f}, "
                f"z={object_in_base.position.z:.3f}"
            )
            
            # Execute pick and place
            self.pick_and_place(object_in_base, object_class='bottle')
            
        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")
    
    def pick_and_place(self, object_pose, object_class='default'):
        """Execute pick and place sequence."""
        
        # Extract coordinates
        x = object_pose.position.x
        y = object_pose.position.y
        z = object_pose.position.z + self.grasp_offset_z  # Adjust for gripper
        
        # 1. Move to approach position (above object)
        self.get_logger().info("Moving to approach position...")
        approach_z = z + self.approach_height
        success = self.robot.move_xyz_no_flip(x, y, approach_z)
        if not success:
            self.get_logger().error("Failed to reach approach position")
            return False
        
        # 2. Open gripper
        self.get_logger().info("Opening gripper...")
        self.robot.gripper_move(0.0)
        
        # 3. Lower to grasp position
        self.get_logger().info("Lowering to object...")
        success = self.robot.move_xyz_no_flip(x, y, z)
        if not success:
            self.get_logger().error("Failed to reach object")
            return False
        
        # 4. Close gripper
        self.get_logger().info("Closing gripper...")
        self.robot.gripper_move(0.8)
        
        # 5. Check grasp
        if not self.robot.check_grasp_success():
            self.get_logger().warn("Grasp failed, aborting")
            self.robot.gripper_move(0.0)
            return False
        
        # 6. Lift object
        self.get_logger().info("Lifting object...")
        self.robot.move_xyz_no_flip(x, y, z + self.approach_height)
        
        # 7. Get bin location
        bin_x, bin_y, bin_z = self.bins.get(object_class, self.bins['default'])
        
        # 8. Move to bin approach
        self.get_logger().info(f"Moving to {object_class} bin...")
        self.robot.move_xyz_no_flip(bin_x, bin_y, bin_z + self.approach_height)
        
        # 9. Lower into bin
        self.robot.move_xyz_no_flip(bin_x, bin_y, bin_z)
        
        # 10. Release object
        self.get_logger().info("Releasing object...")
        self.robot.gripper_move(0.0)
        
        # 11. Retreat
        self.robot.move_xyz_no_flip(bin_x, bin_y, bin_z + self.approach_height)
        
        # 12. Return home
        home = [-1.5707, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]
        self.robot.jmove(home)
        
        self.get_logger().info("Pick and place complete!")
        return True

def main():
    rclpy.init()
    node = VisionGuidedGrasp()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.robot.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Step 5: Camera Calibration & Hand-Eye Calibration

### Why You Need This

Your camera is at a **fixed position** on the desk (from your URDF):
- `cam_left`: x=0.2, y=-0.4, z=0.2 on desk
- `cam_right`: x=0.6, y=0.0, z=0.4 on desk

But you need to verify:
1. **Camera intrinsics** are correct (D435 factory calibration usually good)
2. **Camera extrinsics** (position relative to robot) match your URDF

### Quick Validation Test

```python
#!/usr/bin/env python3
"""
Test camera-robot calibration by pointing at a known location.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class CalibrationTest(Node):
    def __init__(self):
        super().__init__('calibration_test')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
    def test_point(self, x_cam, y_cam, z_cam, camera_frame='cam_left_camera_optical_frame'):
        """Transform a point from camera to base_link."""
        
        point_in_cam = PointStamped()
        point_in_cam.header.frame_id = camera_frame
        point_in_cam.header.stamp = self.get_clock().now().to_msg()
        point_in_cam.point.x = x_cam
        point_in_cam.point.y = y_cam
        point_in_cam.point.z = z_cam
        
        try:
            # Transform to base_link
            point_in_base = self.tf_buffer.transform(
                point_in_cam, 
                'base_link',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            self.get_logger().info(
                f"Camera: ({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f}) → "
                f"Base: ({point_in_base.point.x:.3f}, {point_in_base.point.y:.3f}, "
                f"{point_in_base.point.z:.3f})"
            )
            
            return point_in_base
            
        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")
            return None

def main():
    rclpy.init()
    node = CalibrationTest()
    
    # Test: Object 50cm in front of camera (Z-axis)
    # Camera at desk position (0.2, -0.4, 0.2)
    # Camera looking at (rpy="0 0.5 1.57")
    
    node.test_point(0.0, 0.0, 0.5)  # 50cm forward from camera
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**How to validate:**
1. Place a marker at a known location (e.g., center of desk)
2. Have perception detect it
3. Compare detected position with actual position
4. Adjust camera position in URDF if needed

---

## Step 6: Handling Object Orientation

### If Objects Have Known Orientation

Your colleague can estimate object rotation from:
- Bounding box alignment
- PCA on point cloud
- Deep learning (YOLOv8 pose estimation)

Then you can grasp at the correct angle:

```python
def pick_with_orientation(self, object_pose):
    x = object_pose.position.x
    y = object_pose.position.y
    z = object_pose.position.z
    
    # Convert quaternion to rotation angle (if object is flat on table)
    # For a bottle/box on table, we mainly care about Z-axis rotation
    quat = object_pose.orientation
    
    # Calculate yaw angle (rotation around Z)
    siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1 - 2 * (quat.y**2 + quat.z**2)
    theta = math.atan2(siny_cosp, cosy_cosp)
    
    # Grasp aligned with object
    self.robot.move_xyz_theta_no_flip(x, y, z, theta)
```

### If Objects Are Always Upright

Just use your existing `move_xyz_no_flip()` - gripper points down.

---

## Step 7: Integration Workflow

### Development Phases

**Phase 1: Simulated Detection**
```python
# Publish fake detections to test your grasp node
ros2 topic pub /detected_object_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'cam_left_camera_optical_frame'}, \
    pose: {position: {x: 0.0, y: 0.0, z: 0.5}}}"
```

**Phase 2: Real Detection, Simulated Robot**
- Your colleague publishes real detections
- Your grasp node runs with `use_fake_hardware:=true`
- Validate transforms in RViz

**Phase 3: Real Detection, Real Robot (URSim)**
- Test with URSim before real hardware
- Add safety checks

**Phase 4: Full System**
- Deploy on real robot
- Add error recovery

---

## Step 8: Communication Protocol

### What Your Colleague Should Publish

**Topic 1: Detected Objects**
```
Topic: /detected_objects
Type: vision_msgs/Detection3DArray (or your custom DetectedObjectArray)
Rate: ~10 Hz (when objects detected)
```

**Topic 2: Debug Visualization**
```
Topic: /object_markers
Type: visualization_msgs/MarkerArray
Purpose: Visualize detections in RViz
```

**Topic 3: TF Transforms**
```
TF Tree: cam_left_camera_optical_frame → object_bottle_001
         cam_left_camera_optical_frame → object_box_002
```

### What You Should Provide

**Service: Request Grasp**
```python
# srv/GraspObject.srv
string object_id
---
bool success
string message
```

This allows manual triggering or integration with higher-level planner.

---

## Step 9: Collision Avoidance

### Add Detected Objects to Planning Scene

```python
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

def add_object_to_scene(self, x, y, z, width, height, depth, object_id):
    """Add detected object to MoveIt planning scene."""
    
    collision_object = CollisionObject()
    collision_object.header.frame_id = "base_link"
    collision_object.id = object_id
    
    # Define object shape
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [width, height, depth]
    
    # Position
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    
    collision_object.primitives.append(box)
    collision_object.primitive_poses.append(pose)
    collision_object.operation = CollisionObject.ADD
    
    # Publish to planning scene
    self.collision_pub.publish(collision_object)
    self.get_logger().info(f"Added {object_id} to planning scene")
```

---

## Step 10: Error Handling & Recovery

```python
class RobustGrasp:
    def __init__(self):
        self.max_grasp_attempts = 3
        self.grasp_timeout = 30.0  # seconds
        
    def attempt_grasp(self, object_pose, object_class):
        """Try to grasp with retries."""
        
        for attempt in range(self.max_grasp_attempts):
            self.get_logger().info(f"Grasp attempt {attempt + 1}/{self.max_grasp_attempts}")
            
            success = self.pick_and_place(object_pose, object_class)
            
            if success:
                return True
            
            # Recovery: Return to home and try again
            self.get_logger().warn("Grasp failed, resetting...")
            self.robot.gripper_move(0.0)
            home = [-1.5707, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]
            self.robot.jmove(home)
            
        self.get_logger().error("All grasp attempts failed")
        return False
```

---

## Step 11: Testing Strategy

### Test 1: TF Validation
```bash
# Launch your robot
ros2 launch my_ur_description my_robot.launch.py use_fake_hardware:=true

# Check TF tree
ros2 run tf2_tools view_frames

# Manually publish test object
ros2 run tf2_ros static_transform_publisher 0 0 0.5 0 0 0 \
  cam_left_camera_optical_frame test_object

# Check transform
ros2 run tf2_ros tf2_echo base_link test_object
```

### Test 2: Simulated Grasp
```bash
# Terminal 1: Robot
ros2 launch my_ur_description my_robot.launch.py use_fake_hardware:=true

# Terminal 2: Publish fake detection
ros2 topic pub --once /detected_object_pose geometry_msgs/PoseStamped \
  "{header: {stamp: now, frame_id: 'cam_left_camera_optical_frame'}, \
    pose: {position: {x: 0.0, y: 0.0, z: 0.4}}}"

# Terminal 3: Run grasp node
python3 scripts/vision_guided_grasp.py
```

### Test 3: RViz Visualization

Add to your RViz config:
- TF display (show all frames)
- Marker display (for detected objects)
- Camera display (for RGB/Depth visualization)

---

## Step 12: Performance Optimization

### Grasp Speed vs Accuracy Trade-offs

```python
# Fast grasping (industrial pick-and-place)
self.robot.move_xyz_no_flip(x, y, approach_z)
time.sleep(0.1)  # Minimal settling time
self.robot.gripper_move(0.8)

# Precise grasping (delicate objects)
self.robot.move_xyz_no_flip(x, y, approach_z)
time.sleep(1.0)  # Wait for vibrations to settle
# Add force feedback here if available
self.robot.gripper_move(0.8)
```

### Parallel Processing

```python
import threading

class ParallelGrasp:
    def __init__(self):
        self.detection_queue = []
        self.processing = False
        
    def perception_callback(self, msg):
        """Queue detections for processing."""
        self.detection_queue.append(msg)
        
    def grasp_worker(self):
        """Background thread processes grasps."""
        while rclpy.ok():
            if self.detection_queue and not self.processing:
                self.processing = True
                obj = self.detection_queue.pop(0)
                self.process_grasp(obj)
                self.processing = False
            time.sleep(0.1)
```

---

## Summary & Recommendations

### ✅ Your Understanding is Correct

1. ✅ Add object frames to TF2
2. ✅ Use similar structure to your test script
3. ✅ Transform from camera frame to base_link
4. ✅ Execute pick and place

### 📋 Integration Checklist

- [ ] Define message interface with perception team
- [ ] Verify camera frames in TF tree match physical setup
- [ ] Create vision_guided_grasp.py node
- [ ] Test with simulated detections
- [ ] Validate transforms with known objects
- [ ] Add collision objects to planning scene
- [ ] Test with real perception data + fake hardware
- [ ] Test with URSim
- [ ] Deploy on real robot with safety measures

### 🎯 Key Design Decisions

**1. Coordinate Transform Chain:**
```
Object (in camera frame) 
  → TF2 transform → 
Object (in base_link frame) 
  → Add grasp offset → 
Grasp pose (in base_link frame)
```

**2. Grasp Offset:**
- Camera detects object center
- Gripper TCP is ~15cm below camera-detected point
- Must account for this in Z-axis

**3. Bin Selection:**
- Use object class from YOLO
- Map to predefined bin locations
- Add dynamic bin management if needed

### 🚀 Next Steps

1. **This week:** Get sample detection data from colleague
2. **Test transforms:** Validate camera→base_link transform
3. **Build grasp node:** Start with vision_guided_grasp.py
4. **Integration test:** Run with simulated detections
5. **Real test:** Deploy with actual perception system

---

## Example: Complete Integration Test

```bash
# Terminal 1: Launch robot system
ros2 launch my_ur_description my_robot.launch.py use_fake_hardware:=true

# Terminal 2: Launch your colleague's perception (when ready)
ros2 launch perception_package yolo_detection.launch.py

# Terminal 3: Launch your grasp planner
python3 scripts/vision_guided_grasp.py

# Terminal 4: Trigger a grasp (or let it run autonomously)
ros2 service call /request_grasp my_ur_description/srv/GraspObject \
  "{object_id: 'bottle_001'}"
```

Your architecture is solid! The key is clean interfaces and good testing at each integration stage.
