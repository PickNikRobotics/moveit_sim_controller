cmake_minimum_required(VERSION 2.8.3)
project(moveit_sim_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  ros_control_boilerplate
  rosparam_shortcuts
  roscpp
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES moveit_sim_hw_interface
#  CATKIN_DEPENDS moveit_core ros_control_boilerplate roscpp
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Hardware Interface
add_library(moveit_sim_hw_interface src/moveit_sim_hw_interface.cpp)
target_link_libraries(moveit_sim_hw_interface
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(moveit_sim_hw_main src/moveit_sim_hw_main.cpp)
target_link_libraries(moveit_sim_hw_main
  moveit_sim_hw_interface
  ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS moveit_sim_hw_interface moveit_sim_hw_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
