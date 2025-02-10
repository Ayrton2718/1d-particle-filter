cmake_minimum_required(VERSION 3.8)
project(sim_lcmcl)

find_package(tutrc_ament_node REQUIRED)

tutrc_ament_init()

tutrc_ament_add_executable(sim_lcmcl        src/sim_lcmcl.cpp)
tutrc_ament_add_executable(tf_broadcaster   src/tf_broadcaster.cpp)

# Install launch files.
install(DIRECTORY
    rviz
    DESTINATION share/${PROJECT_NAME}/
)

tutrc_ament_final()
