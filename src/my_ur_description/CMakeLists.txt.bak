cmake_minimum_required(VERSION 3.8)
project(my_ur_description)

find_package(ament_cmake REQUIRED)

# This is the part you need to update/add:
install(DIRECTORY urdf srdf config launch rviz
        DESTINATION share/${PROJECT_NAME})

ament_package()