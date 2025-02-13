cmake_minimum_required(VERSION 3.8)
project(lc_map)
find_package(Eigen3 REQUIRED)

find_package(tutrc_ament REQUIRED)

add_definitions(-DLCMCL_MAP_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/config/aws_small_warehouse")

tutrc_ament_init()

file(GLOB_RECURSE CXX_SOURCES "src/lc_map.cpp")
tutrc_ament_add_library(${PROJECT_NAME} SHARED ${CXX_SOURCES})
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}> # Eigen3用の指定
)

# Install launch files.
install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/
)

tutrc_ament_final()
