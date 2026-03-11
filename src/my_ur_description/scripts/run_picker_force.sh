#!/bin/bash

# Configuration
#SCRIPT_NAME="vision_guided_sorting_brain_once_gripper_client.py"   #with position control only
SCRIPT_NAME="vision_guided_sorting_brain_once_gripper_client_force_contact.py"    #with force control: touch and retract grasping

PORT="/tmp/ttyUR"

echo "Starting Robotic Pick-and-Place Loop..."
echo "Press [CTRL+C] to stop the loop."

while true
do
    echo "------------------------------------------------"
    echo "Step 1: Checking Serial Bridge..."
    
    # Ensure permissions are correct every cycle
    if [ -e "$PORT" ]; then
        sudo chmod 666 "$PORT"
        echo "Port $PORT is ready."
    else
        echo "Waiting for $PORT to appear (check your socat bridge)..."
        #sleep 2
        continue
    fi

    echo "Step 2: Launching Brain..."
    # Run the python script and wait for it to finish a cycle
    python3 "$SCRIPT_NAME"

    echo "Step 3: Cycle Complete. Resetting in 3 seconds..."
    # Brief sleep to let the robot controller and serial port settle
    #sleep 3
done