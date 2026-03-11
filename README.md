# ur12e-robotiq-2f-85-easydesk-scene-camerax2-on-ROS2-humbe-Moveit2
On Ubuntu 24.04, run a Docker of ROS2 Humble, Developing the control and simulation software stack for a robot ur12e with gripper Robotiq 2f-85 on an easydesk workbench, with 2 scene cameras Intel D435.  Software stack includes Moveit2 for motion planning, URSim+URCap for ur12e hardware simulation, Gazebo or issac Sim for physics simulation, ros2 driver and controller for ur12e and 2f-85, RViz for visualization.

***

# Installation Instructions:

Setting up a complete robotics development environment with ROS 2 Humble on Ubuntu 24.04 requires a clean Docker strategy. Since Humble is built for Ubuntu 22.04, Docker acts as a bridge to keep your host system stable while running the robot stack.

1. Host Preparation (Ubuntu 24.04)On your host machine, you need to install Docker and allow the container to open windows (like RViz) on your desktop.
Bash
# Install Docker
sudo apt update && sudo apt install docker.io -y
sudo usermod -aG docker $USER && newgrp docker

# Allow Docker to communicate with your X11 display for RViz
xhost +local:docker

2. The Development Environment (Docker)
Instead of a simple "run" command, use a Dockerfile to bake in all the complicated dependencies for the UR12e, Robotiq, and RealSense cameras.

File: Dockerfile
Dockerfile

FROM osrf/ros:humble-desktop

# Install UR Driver, MoveIt 2, and RealSense meshes
RUN 
    apt-get update && apt-get install -y \
    ros-humble-ur-robot-driver \
    ros-humble-moveit \
    ros-humble-realsense2-description \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
WORKDIR /ros2_ws

Build and Start:
Bash
docker build -t ur12e_dev_env .

docker run -it \
  --name ur_dev_container \
  --net=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/ros2_ws:/ros2_ws \
  ur12e_dev_env

3. File Architecture & Configuration

Inside your /ros2_ws/src/my_ur_description package, your files should be organized as follows:
|File Type       |Path / Role                 |Key Purpose|
|----------------|----------------------------|-----------|
|URDF/Xacro      |urdf/ur_system.xacro        |Combines UR12e, Gripper, Easydesk, and .dae camera meshes.|
|SRDF            |srdf/ur_system.srdf         |Defines "Home" poses and disables self-collisions with walls/desk.|
|YAML            |config/ompl_planning.yaml   |Sets planning time (10s) and precision for tight cage navigation.|
|Launch          |launch/my_robot.launch.py   |Starts robot_state_publisher, move_group, and RViz.|

4. Running the "Fake Hardware" Stack

Once inside the container, you use the Mock Components plugin. This allows MoveIt to think it's talking to a real robot without needing URSim or physical hardware.

    1. Build your package:
    
    Bash

    colcon build --symlink-install
    source install/setup.bash

    2. Launch with Fake Hardware:
    
    Ensure your my_robot.launch.py sets the following parameter for the ur_control.launch.py include:
    
        use_fake_hardware: "true"
    
        fake_sensor_commands: "true"
        
        Bash

        ros2 launch my_ur_description my_robot.launch.py

5. Final Checklist for RViz

When RViz opens, perform these three checks to ensure your system is "intelligent":

    -Goal State: Ensure it's set to <random> or home so you can move the joint sliders.

    -Collision View: In the RobotModel display, toggle "Collision" to see if your camera hit-boxes align with the visual meshes.

    -Planning: Try planning a move from "Home" to "Pick Approach." If it fails, check your OMPL parameters in the launch file.

***

To build this stack, you need a specific directory structure that follows the ROS 2 and MoveIt 2 standards. This ensures that the launch files can find the meshes and the controller manager can find the hardware configurations.

1. Creating the Package
On your host (Ubuntu 24.04) or inside the Docker container, run:

Bash

cd ~/ros2_ws/src

ros2 pkg create --build-type ament_cmake my_ur_description --dependencies 

ur_description moveit_ros_move_group

mkdir -p my_ur_description/{config,launch,meshes,rviz,urdf,srdf}

2. Folder Structure Overview

Here is how your package should look:

Plaintext

my_ur_description/

├── CMakeLists.txt              # Build instructions

├── package.xml                 # Metadata and dependencies

├── config/                     # Configuration parameters

│   ├── kinematics.yaml         # Solver settings

│   ├── ros2_controllers.yaml   # Hardware interface mapping

│   └── ur_controllers.yaml     # Specific UR controller settings

├── launch/                     # Automation scripts

│   └── my_robot.launch.py      # The main entry point

├── rviz/                       # Visualization presets

│   └── my_robot_view.rviz      # Layout and camera angles

├── srdf/                       # Semantic Robot Description

│   └── ur_system.srdf          # Poses and collision rules

└── urdf/                       # Physical Description

    └── ur_system.xacro         # The "Master" assembly file


3. File Functions & Logic

Physical & Logical Description

    -ur_system.xacro: The Master Assembly. It uses <xacro:include> to pull in the UR12e, the Robotiq 2F-85, and your D435 camera macros. It defines how the Easydesk connects to the world and where the robot sits on the desk.

    -ur_system.srdf: The Semantic file. It doesn't define shapes, but rather meanings. It defines named poses (e.g., "Home") and, critically, a Collision Matrix that tells MoveIt to ignore collisions between the robot's base and the Easydesk.

Configuration (YAMLs)

    -kinematics.yaml: Defines which numerical solver MoveIt should use (usually kdl_kinematics_plugin) to calculate how the joints move to reach a 3D coordinate.

    -ros2_controllers.yaml: Configures the Controller Manager. It defines the types of controllers (e.g., JointTrajectoryController) and the sampling rate for the "fake" hardware.

    -ur_controllers.yaml: Specific parameters for the UR12e, such as joint constraints and tolerances for the trajectory follower.

Deployment & Visualization

    -my_robot.launch.py: A Python script that orchestrates everything. It starts the robot_state_publisher (which processes the Xacro), the move_group node (the "brain"), and RViz. It handles logic like use_fake_hardware := true.

    -my_robot_view.rviz: A saved configuration of the RViz GUI. It ensures that when you open the program, the "Motion Planning" panel is already open, the background is the right color, and the cameras are visible.

Package Plumbing

    -CMakeLists.txt: Tells colcon how to install the folders (config, launch, urdf, etc.) into the install/ directory so they can be found at runtime.

    -package.xml: Lists the other ROS 2 packages your project needs (like ur_robot_driver).

4. Build and Source Instructions

Every time you modify the Xacro or YAML files, you must rebuild to update the install folder:

Bash

cd ~/ros2_ws

colcon build --symlink-install --packages-select my_ur_description

source install/setup.bash

# Launch the fake hardware simulation

ros2 launch my_ur_description my_robot.launch.py

***

To build a professional-grade ROS 2 workspace for your UR12e lab on Ubuntu 24.04, you need to understand how these files work together as an "orchestra."Below is the definitive guide to the folder structure and the internal logic of your my_ur_description package.

1. Package Creation & Workspace Layout

Inside your Docker container, create the package. The ament_cmake build type is required for robot descriptions because it handles complex installation of 3D meshes and configuration files better than a pure Python package.

Bash

cd ~/ros2_ws/src

ros2 pkg create --build-type ament_cmake my_ur_description

cd my_ur_description

mkdir urdf srdf launch config rviz

2. File Functions & Logic

|File                    |Purpose                 |Logic     |
|------------------------|------------------------|----------------------------------------------------------------------|
|ur_system.xacro         |The Physical Model      |Combines the UR12e robot, Robotiq gripper, and D435 cameras into one 3D assembly using math and macros.|
|ur_system.srdf          |The Semantic Model      |Defines "Groups" (e.g., ur_arm), "Poses" (e.g., home), and "Virtual Joints." Crucially, it lists which parts can touch without triggering an alarm (Self-Collision).|
|my_robot.launch.py      |The Orchestrator        |A Python script that starts the nodes. It tells MoveIt where the URDF is and launches RViz with your specific saved view.|
|ros2_controllers.yaml   |The Hardware Logic      |Maps MoveIt's abstract commands to specific software "drivers" like the JointTrajectoryController.|
|kinematics.yaml         |The Math Solver         |Configures the Inverse Kinematics (IK) plugin (e.g., KDL or PickNik's BioIK) so the arm knows how to calculate joint angles for a target 3D point.| 
|package.xml             |The Dependency List     |Lists every ROS package your robot needs (e.g., moveit_ros_planning, xacro, ur_description).|
|CMakeLists.txt          |The Installer           |This is the "glue." It tells the compiler which folders (launch, config, urdf) must be copied to the install/ folder so ROS can find them at runtime.|

3. The "Secret Sauce": CMakeLists.txt

Since you are working with Xacro and Moveit, your CMakeLists.txt must explicitly "install" your asset folders. Without this, ros2 launch will error out with "file not found."

Add these lines near the bottom of your CMakeLists.txt:

CMake

# Install directories so they are available in the install/share folder

install(DIRECTORY

  config

  launch

  rviz

  srdf

  urdf

  DESTINATION share/${PROJECT_NAME}

)

4. Build and Launch

Every time you modify the SRDF or Xacro, you must rebuild your workspace to move those changes into the install folder.

Bash

cd ~/ros2_ws

colcon build --symlink-install --packages-select my_ur_description

source install/setup.bash

ros2 launch my_ur_description my_robot.launch.py

***

To finalize your lab setup on Ubuntu 24.04 using Docker and ROS 2 Humble, you need a robust CMakeLists.txt and a clear controller configuration. This "orchestra" ensures that when you move a slider in RViz, the "fake" robot joints respond correctly.

1. The Installer: CMakeLists.txt

This file is the most critical for a description package. It tells the ROS build tool (colcon) which folders to copy from your source directory to the system's install path. If a folder isn't listed here, your launch files will crash with "file not found" errors.

CMake

cmake_minimum_required(VERSION 3.8)

project(my_ur_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(-Wall -Wextra -Wpedantic)

endif()

# Find dependencies

find_package(ament_cmake REQUIRED)

find_package(ur_description REQUIRED)

find_package(ur_robot_driver REQUIRED)


# INSTALL: Copy your asset folders to the install/share space

install(DIRECTORY

  config

  launch

  rviz

  srdf

  urdf

  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

2. The Controller Logic: ros2_controllers.yaml

Since you are using "fake hardware," MoveIt needs to know which software "driver" to use to simulate joint motion. For a UR12e with a 2F-85 gripper, you typically use two controllers.

YAML

controller_manager:

  ros__parameters:

    update_rate: 100  # Hz


    # Define the controllers

    joint_state_broadcaster:

      type: joint_state_broadcaster/JointStateBroadcaster


    ur_manipulator_controller:

      type: joint_trajectory_controller/JointTrajectoryController


    gripper_controller:

      type: position_controllers/GripperActionController


# Configure the Arm Controller

ur_manipulator_controller:

  ros__parameters:

    joints:

      - shoulder_pan_joint

      - shoulder_lift_joint

      - elbow_joint

      - wrist_1_joint

      - wrist_2_joint

      - wrist_3_joint

    command_interfaces:

      - position

    state_interfaces:

      - position

      - velocity


# Configure the Gripper Controller

gripper_controller:

  ros__parameters:

    joint: robotiq_85_left_knuckle_joint



3. Summary of Package Setup Instructions

    -Create Package: Inside your Docker container, run ros2 pkg create --build-type ament_cmake my_ur_description.

    -Organize Files: Move your ur_system.xacro to /urdf, your .srdf to /srdf, and your .yaml files to /config.

    -Update package.xml: Ensure you have <exec_depend> tags for ur_description, joint_state_publisher_gui, and moveit_ros_move_group.

    -Build:

Bash

cd ~/ros2_ws

colcon build --symlink-install --packages-select my_ur_description

source install/setup.bash

Launch: Use your my_robot.launch.py. When the GUI opens, remember to switch the Goal 

State from <current> to <random> to unlock the sliders!


# Why this structure matters

By using the install(DIRECTORY ...) command in CMake, you make your package "portable." You could zip up your install folder and move it to another machine (or another Docker container) and it would run immediately without needing the source code.


***

Here is the concise, step-by-step procedure to launch your full UR12e + Robotiq + Camera system using Fake Hardware (Mock Components). This allows you to plan and visualize in RViz without needing the real robot or URSim.

1. Start & Enter the ROS 2 Container

On your Ubuntu 24.04 host, open a terminal. First, ensure the container can access your screen, then start it.

Bash

# Allow Docker to open GUI windows (RViz)

xhost +local:docker

# Start your Humble container (assuming your workspace is at ~/ros2_ws)

docker run -it \

  --name ur_fake_hw \

  --net=host \

  --privileged \

  -e DISPLAY=$DISPLAY \

  -v /tmp/.X11-unix:/tmp/.X11-unix \

  -v ~/ros2_ws:/ros2_ws \

  ur12e_dev_env


2. Build and Source (Inside the Container)

Once inside the container's bash prompt, you must "register" your package so ROS can see your launch files.

Bash

# Move to the workspace root

cd /ros2_ws

# Build only your description package to save time

colcon build --symlink-install --packages-select my_ur_description

# Source the main ROS 2 installation

source /opt/ros/humble/setup.bash

# Source your local workspace

source install/setup.bash

3. Launch the System

Now, execute the launch file. We pass the use_fake_hardware argument which tells the robot driver to skip the network check for a real IP and instead use the mock_components plugin.

Bash

  ros2 launch my_ur_description my_robot.launch.py \

  use_fake_hardware:=true \

  robot_ip:=123.123.123.123 \

  ur_type:=ur12e


Note: Even with fake hardware, the UR driver requires a dummy robot_ip string to satisfy its internal checks.

4. Interactive Checklist in RViz

When RViz opens, if you cannot move the robot, perform these three quick actions:

    -Add the MotionPlanning Plugin: If the panel isn't visible, click Add -> MotionPlanning.

    -Unlock the Joints: Go to the Planning tab. Change Goal State from <current> to <random>.

    -Visual Check: * Ensure your D435 mesh is visible on the desk.Check that the Robotiq 2F-85 is attached to the tool0 frame.

# Why this works

By setting use_fake_hardware:=true, ROS 2 starts a Mock Hardware Interface. This interface creates an "echo" for joint commands: whenever MoveIt sends a command to move a joint to $90^\circ$, the mock hardware immediately reports back that the joint is now at $90^\circ$. This creates the smooth "ghost" movement you see in the simulator.



### 🛠️ Optional: Mock Gripper Service
To prevent "Gripper Disconnected" errors in URSim:
1. Install dependencies: `pip install pymodbus`
2. Run the mock server: `sudo python3 mock_gripper.py`
3. In URSim, point the Gripper IP to `172.17.0.1`.

1. Identify the Internal IP Addresses
The containers need to talk to each other. Since you aren't using --net=host for the URSim container (based on your docker ps output), they are likely on the default Docker bridge.

Run these commands on your host:

URSim IP: docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' ursim_vnc

ROS 2 IP: docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' tender_stonebraker

Note: Usually, URSim is 172.17.0.2 and ROS 2 is 172.17.0.3.




# Enter the ROS 2 container

docker exec -it tender_stonebraker bash

# Source your workspace

source /opt/ros/humble/setup.bash

source /ros2_ws/install/setup.bash

# Launch pointing to URSim's IP

ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur12e \
  robot_ip:=172.17.0.2 \
  use_fake_hardware:=false \
  launch_rviz:=true \
  initial_joint_controller:=scaled_joint_trajectory_controller

  apt update
apt install -y python3-pymodbus


python3 /ros2_ws/mock_gripper.py


docker exec -it tender_stonebraker bash
source /ros2_ws/install/setup.bash

# Launch MoveIt 2
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur12e \
  launch_rviz:=true \
  use_fake_hardware:=false \
  robot_ip:=172.17.0.2


  TerminalCommandPurpose1python3 mock_gripper.pySatisfies the Robotiq URCap.2ros2 launch ur_robot_driver ur_control.launch.py ...The communication bridge to URSim.3ros2 launch ur_moveit_config ur_moveit.launch.py ...The motion planning GUI and brain.


  ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur12e \
  robot_ip:=172.17.0.2 \
  use_fake_hardware:=false \
  description_package:=my_ur_description \
  description_file:=ur_system.xacro \
  launch_rviz:=true



  