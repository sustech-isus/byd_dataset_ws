cmake_minimum_required(VERSION 2.8.3)
project(byd_sdk)

add_compile_options(-std=c++14)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    rospy
    std_msgs
    )

find_package(Glog REQUIRED)

add_message_files(
    FILES
    ControlInfo.msg
    VehicleStatus.msg
    VehicleOdometry.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    #INCLUDE_DIRS include
    CATKIN_DEPENDS
    message_runtime
    INCLUDE_DIRS
    lib
    #  LIBRARIES twist_control
    #  CATKIN_DEPENDS roscpp rospy std_msgs
    #  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    byd_sdk_node
    src/byd_sdk.cpp
    src/byd_sdk_node.cpp
)



link_directories(
    ${catkin_LIB_DIRS}
)

target_link_libraries(
    byd_sdk_node
    PUBLIC
    ${catkin_LIBRARIES}
    ${GLOG_LIBRARY}
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libadpi.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libcanprocess.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libusbcan.so
    #/usr/local/lib/libgflags.a
)

add_dependencies(
    byd_sdk_node
    byd_sdk_generate_messages
)

