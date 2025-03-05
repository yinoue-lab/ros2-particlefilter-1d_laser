cmake_minimum_required(VERSION 3.8)
project(odom_evaluator)

find_package(tutrc_ament_node REQUIRED)

tutrc_ament_init()

tutrc_ament_create_node(${PROJECT_NAME}_component
    NODE_NAME   ${PROJECT_NAME}
    NODE_PLUGIN "OdomEvaluator"
)

# Install launch files.
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

tutrc_ament_final()

