cmake_minimum_required(VERSION 2.8)

project(AsioSerial)

SET(CMAKE_BUILD_TYPE "Debug")

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -std=c++11 ")

SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -std=c++11 ")

add_subdirectory(ASIOLib)

add_subdirectory(SerialReader)

