cmake_minimum_required(VERSION 3.0.2)
project(moveit_sim_controller)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  ros_control_boilerplate
  rosparam_shortcuts
  roscpp
  roslint
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moveit_sim_hw_interface
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    ros_control_boilerplate
    rosparam_shortcuts
    roscpp
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

# Hardware Interface
add_library(moveit_sim_hw_interface src/moveit_sim_hw_interface.cpp)
target_link_libraries(moveit_sim_hw_interface
  ${catkin_LIBRARIES}
)

## Main executable
add_executable(moveit_sim_hw_main src/moveit_sim_hw_main.cpp)
target_link_libraries(moveit_sim_hw_main
  moveit_sim_hw_interface
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Test for correct C++ source code
roslint_cpp()

#############
## Install ##
#############

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
