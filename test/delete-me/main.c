// Pico4ML Camera Test
// Pico4ML schematic: https://www.arducam.com/downloads/Arducam-Pico4ML-board-Shematics-UC-798_SCH.pdf
// Pins: https://github.com/ArduCAM/RPI-Pico-Cam/tree/master/tflmicro/examples/person_detection_screen

#include "st7735.h"
#include "DEV_Config.h"
#include <stdlib.h>		
#include <stdio.h>
// #include "icm20948.h"
#include "arducam.h"

// Define camera pins in arducam.c

uint8_t image[96*96*2];
// IMU_EN_SENSOR_TYPE result;
// IMU_ST_ANGLES_DATA Angles,  Gyro,  Acce, Magn;

int main(void)
{
	struct arducam_config config;

	// Init UART
	stdio_uart_init();
	sleep_ms(1000);

	// Init LCD
	ST7735_Init();

	// Configure camera pins
	config.sccb = i2c0;					// SCL on GP5, SDA on GP4 means I2C0
	config.sccb_mode = I2C_MODE_16_8;
	config.sensor_address = 0x24;
	config.pin_sioc = PIN_CAM_SIOC;
	config.pin_siod = PIN_CAM_SIOD;
	// config.power_en = PIN_POWER_EN;
	config.pin_resetb = PIN_CAM_RESETB;
	config.pin_xclk = PIN_CAM_XCLK;
	config.pin_vsync = PIN_CAM_VSYNC;
	config.pin_y2_pio_base = PIN_CAM_Y2_PIO_BASE;
	config.pio = pio0;
	config.pio_sm = 0;
	config.dma_channel = 0;

	// Init camera
	arducam_init(&config);

	// Blank out LCD
	ST7735_FillScreen(ST7735_BLUE);

	// Capture image, display it on the LCD
	while(1)
	{
		arducam_capture_frame(&config, image);
		ST7735_DrawImage(0,0, 96, 96,image);
		ST7735_WriteString(101, 30, "85%", Font_16x26, ST7735_GREEN, ST7735_BLUE);   
	}
    
}
