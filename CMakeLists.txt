cmake_minimum_required(VERSION 3.10)
project(so-101-cpp)

# Keep C++17 to match your robotpkg installation requirements
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nlohmann_json REQUIRED)
find_package(pinocchio CONFIG REQUIRED
             PATHS /opt/openrobots/lib/cmake
             NO_DEFAULT_PATH)


add_library(servo_bus servo_bus.cpp)
target_include_directories(servo_bus PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(servo_bus PUBLIC nlohmann_json::nlohmann_json)

add_executable(base_angle base_angle.cpp)
target_link_libraries(base_angle PRIVATE servo_bus)

add_executable(send send.cpp)
target_sources(send PRIVATE cartesian_ik.cpp)
target_compile_definitions(send PRIVATE
    BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
    BOOST_MPL_LIMIT_LIST_SIZE=30)
target_link_libraries(send PRIVATE servo_bus pinocchio::pinocchio_parsers)
configure_file(send_config.json send_config.json COPYONLY)
