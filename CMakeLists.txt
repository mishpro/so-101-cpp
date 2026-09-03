cmake_minimum_required(VERSION 3.10)
project(so-101-cpp)

# Keep C++17 to match your robotpkg installation requirements
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



add_library(servo_bus servo_bus.cpp)
target_include_directories(servo_bus PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(base_angle base_angle.cpp)
target_link_libraries(base_angle PRIVATE servo_bus)

add_executable(send send.cpp)
target_link_libraries(send PRIVATE servo_bus)

