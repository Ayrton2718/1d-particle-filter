cmake_minimum_required(VERSION 3.8)
project(lcmcl_topics)

find_package(Eigen3 REQUIRED)
find_package(tutrc_ament REQUIRED)

tutrc_ament_init()

tutrc_ament_add_library(${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>)

tutrc_ament_final()
