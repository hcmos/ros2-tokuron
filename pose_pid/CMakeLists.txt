cmake_minimum_required(VERSION 3.8)
project(pose_pid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pose_pid_node src/pose_pid_node.cpp)

ament_export_include_directories(include)
# ament_export_libraries(pose_pid)

ament_auto_package(INSTALL_TO_SHARE)
