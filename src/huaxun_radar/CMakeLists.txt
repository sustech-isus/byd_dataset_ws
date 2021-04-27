cmake_minimum_required(VERSION 2.8.3)
project(huaxun_radar)

add_compile_options(-std=c++14)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        pcl_msgs
        )

find_package(PCL REQUIRED)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS
)

## Build talker and listener
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )


link_directories(
        ${catkin_LIB_DIRS}
        )

add_executable(huaxun_radar_node
        src/radar_node.cpp
        src/radar.cpp)


target_link_libraries(huaxun_radar_node
        PRIVATE
        ${catkin_LIBRARIES}
        canlib
       )
