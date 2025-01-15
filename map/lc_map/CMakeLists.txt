cmake_minimum_required(VERSION 3.8)
project(lc_map)
find_package(Eigen3 REQUIRED)

find_package(tutrc_ament REQUIRED)

add_definitions(-DLCMCL_MAP_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/config/abu2024_blue")

tutrc_ament_init()

# set(tag_logger_c
#   src/tl_file.cpp
# )

# add_library(${PROJECT_NAME} ${tag_logger_c})
# # ament_export_libraries(${PROJECT_NAME})


# tutrc_ament_add_library(${PROJECT_NAME} PUBLIC)

file(GLOB_RECURSE CXX_SOURCES "src/lc_map.cpp")
tutrc_ament_add_library(${PROJECT_NAME} SHARED ${CXX_SOURCES})
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}> # Eigen3用の指定
)

tutrc_ament_final()
