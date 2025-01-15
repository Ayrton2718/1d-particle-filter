cmake_minimum_required(VERSION 3.8)
project(lcmcl)

find_package(tutrc_ament_node REQUIRED)
find_package(Eigen3 REQUIRED)

tutrc_ament_init()

tutrc_ament_create_node(${PROJECT_NAME}_component
    NODE_NAME   ${PROJECT_NAME}
    NODE_PLUGIN "mcl::Lcmcl"
)

tutrc_ament_final()
