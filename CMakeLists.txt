cmake_minimum_required(VERSION 3.13.1)

set(MODEL_FOLDER source)
set(EI_SDK_FOLDER source/edge-impulse-sdk)

include(pico_sdk_import.cmake)

project(app C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

pico_sdk_init()

# TODO: undefined reference in main to arducam functions
# CMake not linking/including them properly?

# Main executable
add_executable(app
	source/arducam/arducam.c
	source/main.cpp
)

# Add Edge Impulse SDK (recursive CMakeLists.txt)
add_subdirectory(source/edge-impulse)

# # Add arducam library
# target_include_directories(app
# 	PUBLIC
# 	source/arducam
# )

# Generate PIO header
pico_generate_pio_header(app 
	${CMAKE_CURRENT_LIST_DIR}/source/arducam/image.pio
)

# Link to libraries
target_link_libraries(app
	pico_stdlib
	hardware_dma
	hardware_i2c
	hardware_pio
	hardware_pwm
	hardware_spi
	pico_multicore
)

# enable usb output, disable uart output
pico_enable_stdio_usb(app 1)
pico_enable_stdio_uart(app 0)

# add all sources to the project
target_sources(app PRIVATE ${SOURCE_FILES})

pico_add_extra_outputs(app)