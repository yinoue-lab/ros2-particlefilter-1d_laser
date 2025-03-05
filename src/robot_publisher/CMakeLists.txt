cmake_minimum_required(VERSION 3.8)
project(robot_publisher)

find_package(tutrc_ament REQUIRED)

tutrc_ament_init()

# Install launch files.
install(DIRECTORY
    launch xacro
    DESTINATION share/${PROJECT_NAME}/
)

tutrc_ament_final()
