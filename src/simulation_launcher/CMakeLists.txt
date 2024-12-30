cmake_minimum_required(VERSION 3.8)
project(simulation_launcher)

find_package(tutrc_ament_node REQUIRED)

tutrc_ament_init()

# Install launch files.
install(DIRECTORY
    launch models urdf
    DESTINATION share/${PROJECT_NAME}/
)

tutrc_ament_final()