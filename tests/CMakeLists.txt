add_executable(test_arm_model test_arm_model.cpp)
target_link_libraries(test_arm_model PRIVATE arm_model)
so101_configure_target(test_arm_model)
add_test(NAME arm_model COMMAND test_arm_model
         WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

add_executable(test_control_config test_control_config.cpp)
target_link_libraries(test_control_config PRIVATE arm_model)
so101_configure_target(test_control_config)
add_test(NAME control_config COMMAND test_control_config
         WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# Транспорт проверяется на псевдо-порту: так воспроизводится обрыв USB, который
# на живом железе случается сам собой и который нельзя увидеть в обычном
# прогоне. openpty лежит в libutil.
add_executable(test_servo_bus test_servo_bus.cpp)
target_link_libraries(test_servo_bus PRIVATE servo_bus util)
so101_configure_target(test_servo_bus)
add_test(NAME servo_bus COMMAND test_servo_bus
         WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# Рисование окружности проверяется без робота: геометрия и файл плоскости —
# на чистых данных, цикл по траектории и захват — на заглушке руки.
add_executable(test_draw_circle test_draw_circle.cpp)
target_link_libraries(test_draw_circle PRIVATE drawing_execution)
so101_configure_target(test_draw_circle)
add_test(NAME draw_circle COMMAND test_draw_circle
         WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# Тесты читают реально поставляемый send_config.json из каталога сборки:
# это проверка настоящей конфигурации, а не её копии в коде. Калибровки
# моторов среди тестовых входных данных нет — она живёт в регистрах.
foreach(test_target test_arm_model test_control_config test_draw_circle)
    add_custom_command(TARGET ${test_target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_SOURCE_DIR}/send_config.json
                ${CMAKE_CURRENT_BINARY_DIR}/send_config.json
        COMMENT "Refreshing test inputs")
endforeach()