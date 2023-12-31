cmake_minimum_required(VERSION 3.14)

# Make project data accessible to compiler
add_compile_definitions(APP_NAME="${APP_NAME}")
add_compile_definitions(APP_VERSION="${APP_VERSION_NUMBER}")
add_compile_definitions(BUILD_NUM=${BUILD_NUMBER})

# Include app source code file(s)
add_executable(${APP_NAME}
${APP_SRC_DIRECTORY}/main.c
${APP_SRC_DIRECTORY}/ssd1306.c
)

# Link to built libraries
target_link_libraries(${APP_NAME} LINK_PUBLIC
    pico_stdlib
    hardware_i2c
    hardware_adc
    FreeRTOS)

# Enable/disable STDIO via USB and UART
pico_enable_stdio_usb(${APP_NAME} 1)
pico_enable_stdio_uart(${APP_NAME} 0)

# Enable extra build products
pico_add_extra_outputs(${APP_NAME})
