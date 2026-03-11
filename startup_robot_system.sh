#!/bin/bash
set -e

echo "==============================="
echo "   Vision-Guided Robot System"
echo "   Auto Start Script (ROS2)"
echo "==============================="

export ROS_DOMAIN_ID=43

### 1. Source ROS2 + workspace
source /opt/ros/humble/setup.bash
source ~/humble_ws/install/setup.bash

### 2. Start UR driver (background)
echo "[1/6] Starting UR Driver..."
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur12e \
  robot_ip:=192.168.2.2 \
  use_fake_hardware:=false \
  kinematics_params:=./src/my_ur_description/config/ur12e_calibration.yaml \
  initial_joint_controller:=scaled_joint_trajectory_controller \
  > /tmp/ur_driver.log 2>&1 &

sleep 5


### 3. Activate scaled_joint_trajectory_controller
echo "[2/6] Activating scaled_joint_trajectory_controller..."
ros2 control switch_controllers \
  --activate scaled_joint_trajectory_controller \
  --deactivate joint_trajectory_controller || true

sleep 2


### 4. Start MoveIt 2 (background)
echo "[3/6] Launching MoveIt2..."
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur12e \
  launch_rviz:=true \
  > /tmp/moveit.log 2>&1 &

sleep 5


### 5. MoveIt Scene Setup
echo "[4/6] Running MoveIt scene setup..."
python3 ~/humble_ws/src/my_ur_description/scripts/setup_moveit_scene2.py \
  > /tmp/moveit_scene.log 2>&1 &

sleep 2



echo ""
echo "===================================================="
echo " All systems started successfully!"
echo " Logs: /tmp/ur_driver.log, /tmp/moveit.log, /tmp/socat.log"
echo "===================================================="