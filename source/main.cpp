/**
 * Read from Arducam camera (Himax HM01B0) in grayscale, resize image to 96x96, 
 * run inference with resized image as input, and print results over serial.
 * 
 * Copyright (c) 2023 EdgeImpulse, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// RP2040 libraries
#include <hardware/gpio.h>
#include <hardware/uart.h>
#include <pico/stdio_usb.h>
#include <stdio.h>

// Edge Impulse SDK
#include "edge-impulse/edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse/edge-impulse-sdk/dsp/image/image.hpp"

// Debug settings
#define LED_DEBUG
// #define ENABLE_LCD

// Arducam camera library
#include "arducam/arducam.h"

// Arducam LCD libraries
#ifdef ENABLE_LCD
# include "lcd/st7735.h"
# include "lcd/fonts.h"
#endif

// Give access to the DSP functions in the Edge Impulse SDK
using namespace ei::image::processing;

// Settings
const uint LED_PIN = 25;
const int CAM_WIDTH = 324;
const int CAM_HEIGHT = 324;
const int IMG_WIDTH = 96;
const int IMG_HEIGHT = 96;
#ifdef ENABLE_LCD
const int LCD_WIDTH = 80;
const int LCD_HEIGHT = 80;
#endif
const bool NN_DEBUG = false;

// Globals
static uint8_t image_buf[CAM_WIDTH * CAM_HEIGHT];

// Callback function declaration
static int get_signal_data(size_t offset, size_t length, float *out_ptr);

// Main entrypoint
int main()
{
  signal_t signal;            // Wrapper for raw input buffer
  ei_impulse_result_t result; // Used to store inference output
  EI_IMPULSE_ERROR res;       // Return code from inference
#ifdef ENABLE_LCD
  uint8_t display_buf[LCD_WIDTH * LCD_HEIGHT * 2];
#endif

  // Init LED
#ifdef LED_DEBUG
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
#endif

  // Init USB (for printing), wait to allow for connection
  stdio_usb_init();
  sleep_ms(2000);
  ei_printf("Person detection demo\r\n");

  // Calculate the length of the buffer
  size_t buf_len = IMG_WIDTH * IMG_HEIGHT;

  // Make sure that the length of the buffer matches expected input length
  if (buf_len != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    while(1)
    {
      ei_printf("ERROR: The size of the input buffer is not correct.\r\n");
      ei_printf("Expected %d items, but got %d\r\n", 
              EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, 
              (int)buf_len);
      sleep_ms(2000);
    }
  }

  // Assign callback function to fill buffer used for preprocessing/inference
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data = &get_signal_data;

  // Set camera config
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

  // Init camera
	arducam_init(&config);

  // Init LCD
#ifdef ENABLE_LCD
  ST7735_Init();
  ST7735_DrawImage(0, 0, 80, 160, arducam_logo);
  sleep_ms(2000);
#endif

  // Main loop
  while (1)
  {
    // Capture image
#ifdef LED_DEBUG
    gpio_put(PIN_LED, 1);
#endif
    arducam_capture_frame(&config);

    // Resize image (in place) using built-in EI function (bilinear interpolation)
    resize_image(image_buf, 
                  CAM_WIDTH, 
                  CAM_HEIGHT, 
                  image_buf, 
                  IMG_WIDTH, 
                  IMG_HEIGHT, 
                  MONO_B_SIZE);
    // Turn off LED (to show that image capture is done)
#ifdef LED_DEBUG
    gpio_put(PIN_LED, 0);
#endif

    // Perform DSP pre-processing and inference
    res = run_classifier(&signal, &result, NN_DEBUG);

    // Print return code and how long it took to perform inference
    ei_printf("run_classifier returned: %d\r\n", res);
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n", 
            result.timing.dsp, 
            result.timing.classification, 
            result.timing.anomaly);

    // Print classification results
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }

    // Display image on LCD
#ifdef ENABLE_LCD

    // Resize image to fit in LCD
    resize_image(image_buf, 
                  IMG_WIDTH, 
                  IMG_HEIGHT, 
                  image_buf, 
                  LCD_WIDTH, 
                  LCD_HEIGHT, 
                  MONO_B_SIZE);

    // Convert image to RGB565
    uint16_t ix = 0;
    uint8_t pixel_gray;
    uint16_t pixel_rgb565;
    for (int y = 0; y < CAM_HEIGHT; y++)
    {
      for (int x = 0; x < CAM_WIDTH; x++)
      {
        pixel_gray = image_buf[(y * CAM_WIDTH) + x];
        pixel_rgb565 = ST7735_COLOR565(pixel_gray, pixel_gray, pixel_gray);
        display_buf[ix++] = (uint8_t)(pixel_rgb565 >> 8) & 0xFF;
        display_buf[ix++] = (uint8_t)(pixel_rgb565) & 0xFF;
      }
    }

    // Display on LCD
    ST7735_DrawImage(0, 0, 80, 160, display_buf);
#endif // ENABLE_LCD
  }
}

// Callback: fill a section of the out_ptr buffer when requested
static int get_signal_data(size_t offset, size_t length, float *out_ptr) 
{
  uint8_t c;
  float pixel_f;

  // Loop through requested pixels, copy grayscale to RGB channels
  for (size_t i = 0; i < length; i++) 
  {
    c = (image_buf + offset)[i];
    pixel_f = (c << 16) + (c << 8) + c;
    out_ptr[i] = pixel_f;
  }

  return EIDSP_OK;
}