cmake_minimum_required(VERSION 2.8.3)
project(moveit_sim_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  ros_control_boilerplate
  roscpp
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES sim_hardware_interface
#  CATKIN_DEPENDS moveit_core ros_control_boilerplate roscpp
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Hardware Interface
add_library(sim_hardware_interface src/sim_hardware_interface.cpp)
target_link_libraries(sim_hardware_interface
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(sim_hardware_main src/sim_hardware_main.cpp)
target_link_libraries(sim_hardware_main
  sim_hardware_interface
  ${catkin_LIBRARIES}
)

