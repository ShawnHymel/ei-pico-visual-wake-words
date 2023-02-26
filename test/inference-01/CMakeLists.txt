cmake_minimum_required(VERSION 3.12)

#include(pico-sdk/pico_sdk_init.cmake)
# Pull in PICO SDK (must be before project)
include(pico_sdk_import.cmake)

project(rp2040_arducam C CXX ASM)

pico_sdk_init()

add_executable(arducam_firmware
	source/arducam/arducam.c
	source/main.cpp
	source/lcd/st7735.c
	source/lcd/fonts.c
	source/lcd/DEV_Config.c
)

pico_generate_pio_header(arducam_firmware 
	${CMAKE_CURRENT_LIST_DIR}/source/arducam/image.pio
)

target_link_libraries(arducam_firmware
	pico_stdlib
	hardware_dma
	hardware_i2c
	hardware_pio
	hardware_pwm
	hardware_spi
	pico_multicore
)

# Enable serial over USB
pico_enable_stdio_usb(arducam_firmware 1)

# Build extra binaries (e.g. uf2)
pico_add_extra_outputs(arducam_firmware)
