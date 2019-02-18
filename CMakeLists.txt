cmake_minimum_required(VERSION 3.1)
project(frontier_exploration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    tf
    roborts_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_msg_test
#  CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${OpenCV_INCLUDE_DIRECTORIES}
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(frontier_exploration_node
    src/frontier_exploration_node.cpp
    src/frontier_exploration.cpp
    src/frontier_point.h
    src/frontier_detection_opencv.h
    )

target_link_libraries(frontier_exploration_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

