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

# Тесты читают реально поставляемый send_config.json из каталога сборки:
# это проверка настоящей конфигурации, а не её копии в коде. Калибровки
# моторов среди тестовых входных данных нет — она живёт в регистрах.
foreach(test_target test_arm_model test_control_config)
    add_custom_command(TARGET ${test_target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_SOURCE_DIR}/send_config.json
                ${CMAKE_CURRENT_BINARY_DIR}/send_config.json
        COMMENT "Refreshing test inputs")
endforeach()