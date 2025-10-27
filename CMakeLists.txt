cmake_minimum_required(VERSION 3.28)

# Pull in SDK (must be before project())
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(address_resolver C CXX ASM)

set(PICO_FLASH_SIZE 2097152)  # 2MB total
set(PICO_FLASH_BINARY_END_OFFSET 0x1F000)  # Reserve last 4KB for NVS (0x200000 - 0x1000)

# Enable the Pico SDK
pico_sdk_init()

# Create executable
add_executable(address_resolver address_resolver.cpp settings/Settings.cpp)

add_subdirectory(nvs_lib)

# Generate PIO header
pico_generate_pio_header(address_resolver 
  ${CMAKE_CURRENT_LIST_DIR}/pio_rx.pio
  ${CMAKE_CURRENT_LIST_DIR}/pio_tx.pio
)

# Link libraries
target_link_libraries( address_resolver PRIVATE 
    pico_stdlib
    hardware_pio 
    hardware_flash 
    NVSOnboard
    hardware_pwm
)

# (Optional) Include current dir if needed
target_include_directories( address_resolver PRIVATE 
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/settings
)

# Add USB output, UF2, etc.
pico_add_extra_outputs(address_resolver)

pico_enable_stdio_usb(address_resolver 1)
pico_enable_stdio_uart(address_resolver 0)
