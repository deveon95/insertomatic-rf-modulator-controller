# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(insertomatic-pico C CXX ASM)

pico_sdk_init()

# Add executable. Default name is the project name, 
add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
add_subdirectory(./lib/pi-pico-LCD/lcd_display build)

pico_add_extra_outputs(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} pico_stdlib lcd_display)
