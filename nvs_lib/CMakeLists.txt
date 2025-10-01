# Minimum version is optional if included in the main CMakeLists.txt
# cmake_minimum_required(VERSION 3.28)

# Define a static library from the source file(s)

add_library(NVSOnboard
    NVSOnboard.cpp
    NVSOnboard.h
)

target_link_libraries(NVSOnboard PUBLIC pico_stdlib hardware_flash)

# Specify include directories for this library
target_include_directories(NVSOnboard PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# (Optional) Set any compile options, warnings, or dependencies here
# target_compile_options(NVSOnboard PRIVATE -Wall -Wextra)
# Force older, safe debug info
add_compile_options(-g0 -fno-asynchronous-unwind-tables -fno-unwind-tables)
