#!/bin/bash
ros2 launch my_ur_description my_robot.launch.py \
  ur_type:=ur12e \
  robot_ip:=127.0.0.1 \
  use_fake_hardware:=false