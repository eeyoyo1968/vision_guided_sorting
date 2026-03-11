FROM osrf/ros:humble-desktop

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies for UR, Robotiq, and RealSense
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-ur-robot-driver \
    ros-humble-moveit \
    ros-humble-robotiq-description \
    ros-humble-gripper-controllers \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-realsense2-description \
    git \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install specific Python libraries
RUN pip3 install minimalmodbus

# Set up the workspace
WORKDIR /ros2_ws

# --- ENVIRONMENT RESTORATION FIXES ---

# 1. Force FastRTPS to fix Interactive Marker/IK communication issues in Docker
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RUN echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc

# 2. Add Graphics support variables
ENV QT_X11_NO_MITSHM=1

# 3. Source ROS and workspace automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]