#!/bin/bash
set -e
source ~/humble_ws/install/setup.bash

echo "Choose a test:"
echo "1) Pick Once"
echo "2) Pick with force control"
echo "3) Go Home"
echo "4) Test Move (sync/async)"
echo "5) Cartesian move"
echo "6) Repeated sorting"
read -p "Selection: " choice

cd ~/humble_ws/src/my_ur_description/scripts

case $choice in
  1)
    python3 vision_guided_sorting_brain_once_gripper_client.py
    ;;
  2)
    python3 vision_guided_sorting_brain_once_gripper_client_force_contact.py
    ;;
  3)
    python3 home.py
    ;;
  4)
    python3 move_plan_sync_async_works.py
    ;;
  5)
    python3 move_plan_xyz_theta.py
    ;;
  6)
    ./run_picker.sh
    ;;
  *)
    echo "Invalid choice."
    ;;
esac