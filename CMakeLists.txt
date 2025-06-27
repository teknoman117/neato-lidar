cmake_minimum_required(VERSION 3.14)
project(neato-lidar VERSION 0.1 LANGUAGES CXX)

find_package(OpenCV REQUIRED)

add_executable(neato-lidar neato-lidar.cpp serial.cpp)

target_link_libraries(neato-lidar PRIVATE opencv_highgui)