cmake_minimum_required(VERSION 3.8)
project(sim_lcmcl)

find_package(tutrc_ament_node REQUIRED)

tutrc_ament_init()

tutrc_ament_add_executable(sim_lcmcl    src/sim_lcmcl.cpp)
tutrc_ament_add_executable(rviz_bridge  src/rviz_bridge.cpp)
tutrc_ament_add_executable(lcmcl_tf_publisher  src/lcmcl_tf_publisher.cpp)

# Install launch files.
install(DIRECTORY
    rviz
    DESTINATION share/${PROJECT_NAME}/
)

tutrc_ament_final()
