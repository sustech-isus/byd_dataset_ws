cmake_minimum_required(VERSION 2.8.3)
project(huaxun_radar)

add_compile_options(-std=c++14)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    huaxun_radar_msgs
    )

## Declare a catkin package
catkin_package(
INCLUDE_DIRS lib
)

## Build talker and listener
include_directories(${catkin_INCLUDE_DIRS} )
link_directories(lib ${catkin_LIB_DIRS}  )
add_executable(huaxun_radar_node
        src/radar_node.cpp
        src/radar.cpp)
target_link_libraries(huaxun_radar_node ${catkin_LIBRARIES} controlcan)
add_dependencies(huaxun_radar_node
    huaxun_radar_msgs_generate_messages
)
