/**
 * From: https://github.com/ArduCAM/RPI-Pico-Cam/tree/master/rp2040_hm01b0_st7735
 * 
 * Inference: looking for 96x96 RGB images as input
 * 
 * TODO:
 *  - Print out image buffer to verify image format
 *  - Capture/scale 96x96 RGB
 */

// 
// 

#include <stdio.h>
#include "pico/stdlib.h"
#include <tusb.h>
#include "pico/multicore.h"
#include "arducam/arducam.h"
#include "lcd/st7735.h"
#include "lcd/fonts.h"

uint8_t image_buf[324*324];
uint8_t displayBuf[80*160*2];
uint8_t header[2] = {0x55,0xAA};

#define FLAG_VALUE 123

void core1_entry() {
	multicore_fifo_push_blocking(FLAG_VALUE);

	uint32_t g = multicore_fifo_pop_blocking();

	if (g != FLAG_VALUE)
		printf("Hmm, that's not right on core 1!\n");
	else
		printf("It's all gone well on core 1!\n");

	gpio_init(PIN_LED);
	gpio_set_dir(PIN_LED, GPIO_OUT);

	ST7735_Init();
	ST7735_DrawImage(0, 0, 80, 160, arducam_logo);

	struct arducam_config config;
	config.sccb = i2c0;
	config.sccb_mode = I2C_MODE_16_8;
	config.sensor_address = 0x24;
	config.pin_sioc = PIN_CAM_SIOC;
	config.pin_siod = PIN_CAM_SIOD;
	config.pin_resetb = PIN_CAM_RESETB;
	config.pin_xclk = PIN_CAM_XCLK;
	config.pin_vsync = PIN_CAM_VSYNC;
	config.pin_y2_pio_base = PIN_CAM_Y2_PIO_BASE;

	config.pio = pio0;
	config.pio_sm = 0;

	config.dma_channel = 0;
	config.image_buf = image_buf;
	config.image_buf_size = sizeof(image_buf);

	arducam_init(&config);
	while (true) {

		sleep_ms(2000);

		gpio_put(PIN_LED, 1);
		arducam_capture_frame(&config);

		
		// Print buffer
		for (uint32_t i = 0; i < config.image_buf_size; i++)
		{
			printf("%i", image_buf[i]);
			if (i < config.image_buf_size - 1)
			{
				printf(", ");
			}
		}
		printf("\r\n");

		gpio_put(PIN_LED, 0);

		while(1);

	//   uint16_t index = 0;
	//   for (int y = 0; y < 160; y++) {
	//     for (int x = 0; x < 80; x++) {
    //           uint8_t c = image_buf[(2+320-2*y)*324+(2+40+2*x)];
    //           uint16_t imageRGB   = ST7735_COLOR565(c, c, c);
    //           displayBuf[index++] = (uint8_t)(imageRGB >> 8) & 0xFF;
    //           displayBuf[index++] = (uint8_t)(imageRGB)&0xFF;
    //         }
	//   }
	//   ST7735_DrawImage(0, 0, 80, 160, displayBuf);

	  printf("Hello\r\n");
	}
}

#include "hardware/vreg.h"

int main() {
  int loops=20;
  stdio_init_all();
  while (!tud_cdc_connected()) { sleep_ms(100); if (--loops==0) break;  }

  printf("tud_cdc_connected(%d)\n", tud_cdc_connected()?1:0);

  vreg_set_voltage(VREG_VOLTAGE_1_30);
  sleep_ms(1000);
  set_sys_clock_khz(250000, true);

  multicore_launch_core1(core1_entry);

  uint32_t g = multicore_fifo_pop_blocking();

  if (g != FLAG_VALUE)
    printf("Hmm, that's not right on core 0!\n");
  else {
    multicore_fifo_push_blocking(FLAG_VALUE);
    printf("It's all gone well on core 0!\n");
  }

  while (1)
    tight_loop_contents();
}
