add_executable(ir_sensor_demo
    main.c
    functions.c
    motor_control.c
)

target_include_directories(ir_sensor_demo PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(ir_sensor_demo
    pico_stdlib
    hardware_gpio
    hardware_adc
    hardware_timer
    hardware_pwm
)

pico_enable_stdio_usb(ir_sensor_demo 1)
pico_enable_stdio_uart(ir_sensor_demo 0)

pico_add_extra_outputs(ir_sensor_demo)




