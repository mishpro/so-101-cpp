cmake_minimum_required(VERSION 3.20)
project(so-101-cpp LANGUAGES CXX)

# Keep C++17 to match your robotpkg installation requirements
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Без явного типа сборки CMake не добавляет флагов оптимизации вовсе, и
# итерации DLS считаются в -O0.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
                 Debug Release RelWithDebInfo MinSizeRel)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(SO101_WERROR "Treat compiler warnings as errors" OFF)

# ВАЖНО: `address` несовместим с prebuilt pinocchio из /opt/openrobots.
# Eigen отключает свой fast-path malloc при __SANITIZE_ADDRESS__
# (Eigen/src/Core/util/Memory.h:33), поэтому наш ASan-инструментированный
# translation unit освобождает память через handmade_aligned_free, а
# libpinocchio_parsers.so выделяет её обычным malloc/realloc. Любой вызов
# buildModel даёт ложное срабатывание ASan — воспроизводится на программе из
# пяти строк, без кода проекта. UBSan такой проблемы не имеет.
set(SO101_SANITIZER "none" CACHE STRING
    "Sanitizers: none | undefined | address | address,undefined")
set_property(CACHE SO101_SANITIZER PROPERTY STRINGS
             none undefined address "address,undefined")

# Предупреждения задаются на цель, а не глобально: Eigen и Boost приносят
# собственные предупреждения из своих заголовков.
set(SO101_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor)
if(SO101_WERROR)
    list(APPEND SO101_WARNINGS -Werror)
endif()

set(SO101_SANITIZER_FLAGS -g -fno-omit-frame-pointer)
if(NOT SO101_SANITIZER STREQUAL "none")
    list(APPEND SO101_SANITIZER_FLAGS -fsanitize=${SO101_SANITIZER})
endif()

function(so101_configure_target target)
    target_compile_options(${target} PRIVATE ${SO101_WARNINGS} ${SO101_SANITIZER_FLAGS})
    if(NOT SO101_SANITIZER STREQUAL "none")
        target_link_options(${target} PUBLIC -fsanitize=${SO101_SANITIZER})
    endif()
endfunction()

find_package(nlohmann_json REQUIRED)
find_package(pinocchio REQUIRED)

add_library(servo_bus servo_bus.cpp)
target_include_directories(servo_bus PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
so101_configure_target(servo_bus)

# nlohmann_json нужен control_config.cpp, а не самому servo_bus: транспорт
# не знает про JSON.
#
# control_config вынесен в отдельную библиотеку, потому что читать send_config.json
# должен в том числе base_angle, а тянуть ради этого pinocchio незачем.
add_library(control_config control_config.cpp)
target_include_directories(control_config PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(control_config PUBLIC nlohmann_json::nlohmann_json)
so101_configure_target(control_config)

add_library(arm_model arm_model.cpp)
target_include_directories(arm_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(arm_model PUBLIC servo_bus pinocchio::pinocchio
                                   control_config)
so101_configure_target(arm_model)

add_executable(base_angle base_angle.cpp)
target_link_libraries(base_angle PRIVATE control_config servo_bus)
so101_configure_target(base_angle)

add_executable(send send.cpp)
target_link_libraries(send PRIVATE arm_model)
so101_configure_target(send)

add_executable(cartesian_control cartesian_control.cpp)
target_link_libraries(cartesian_control PRIVATE arm_model)
so101_configure_target(cartesian_control)

# Контракт руки для логики рисования: читать положения и отправлять цели.
# Железа не знает — от него зависит только интерфейс.
add_library(arm_feedback INTERFACE)
target_include_directories(arm_feedback INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(arm_feedback INTERFACE servo_bus)

# Что и где рисуем: плоскость и окружность. Без железа — значит, проверяется
# в тестах без робота.
add_library(drawing drawing.cpp)
target_include_directories(drawing PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(drawing PUBLIC arm_model nlohmann_json::nlohmann_json)
so101_configure_target(drawing)

# Как рисуем: захват плоскости и обход траектории.
add_library(drawing_execution drawing_execution.cpp)
target_include_directories(drawing_execution PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(drawing_execution PUBLIC arm_model drawing arm_feedback)
so101_configure_target(drawing_execution)

# Единственное место, где логика рисования встречается с шиной.
add_library(serial_arm serial_arm.cpp)
target_include_directories(serial_arm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(serial_arm PUBLIC arm_feedback control_config)
so101_configure_target(serial_arm)

add_executable(draw_circle draw_circle.cpp)
target_link_libraries(draw_circle PRIVATE drawing_execution serial_arm
                                       control_config)
so101_configure_target(draw_circle)

# Копия конфига обновляется при каждой сборке, а не только при конфигурации:
# иначе правка send_config.json не попадала бы в build/ до перезапуска cmake.
# Калибровки моторов здесь нет: она живёт в регистрах сервоприводов, а её
# перенос в URDF делает tools/sync_urdf_limits.py, читая те же регистры.
add_custom_command(TARGET cartesian_control POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/send_config.json
            ${CMAKE_CURRENT_BINARY_DIR}/send_config.json
    COMMENT "Refreshing config in the build directory")

# То же для программы рисования: она читает тот же конфиг.
add_custom_command(TARGET draw_circle POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/send_config.json
            ${CMAKE_CURRENT_BINARY_DIR}/send_config.json
    COMMENT "Refreshing config in the build directory")

include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()