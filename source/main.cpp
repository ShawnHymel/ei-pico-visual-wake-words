// RP2040 libraries
#include <hardware/gpio.h>
#include <hardware/uart.h>
#include <pico/stdio_usb.h>
#include <stdio.h>

// Arducam camera library
#include "arducam/arducam.h"

// Edge Impulse SDK
#include "edge-impulse/edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse/edge-impulse-sdk/dsp/image/image.hpp"

// Give access to the DSP functions in the Edge Impulse SDK
using namespace ei::image::processing;

// Settings
const uint LED_PIN = 25;
const int CAM_WIDTH = 324;
const int CAM_HEIGHT = 324;
const int IMG_WIDTH = 96;
const int IMG_HEIGHT = 96;

// // Edge Impulse callback to fill inference buffer
// int raw_feature_get_data(size_t offset, size_t length, float *out_ptr)
// {
//   memcpy(out_ptr, features + offset, length * sizeof(float));
//   return 0;
// }

// Globals
static uint8_t image_buf[CAM_WIDTH * CAM_HEIGHT];

int main()
{
  // Init LED
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  // Init USB (for printing)
  stdio_usb_init();

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

  // ***TEST***
  sleep_ms(2000);

  // Capture image
  gpio_put(PIN_LED, 1);
  arducam_capture_frame(&config);

  // Resize image (in place) using built-in EI function (bilinear interpolation)
  resize_image(image_buf, 
                CAM_WIDTH, 
                CAM_HEIGHT, 
                image_buf, 
                IMG_WIDTH, 
                IMG_HEIGHT, 
                MONO_B_SIZE);

  // Print buffer
  for (uint32_t i = 0; i < (IMG_WIDTH * IMG_HEIGHT); i++)
  {
    printf("%i", image_buf[i]);
    if (i < config.image_buf_size - 1)
    {
      printf(", ");
    }
  }
  printf("\r\n");

  // Turn off LED (to show that image capture is done)
  gpio_put(PIN_LED, 0);

  while(1);

//   ei_impulse_result_t result = {nullptr};
//   while (true)
//   {
//     ei_printf("Edge Impulse standalone inferencing (Raspberry Pi Pico)\n");

//     if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
//     {
//       ei_printf("The size of your 'features' array is not correct. Expected %d items, but had %u\n",
//                 EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
//       return 1;
//     }

//     while (1)
//     {
//       // blink LED
//       gpio_put(LED_PIN, !gpio_get(LED_PIN));

//       // the features are stored into flash, and we don't want to load everything into RAM
//       signal_t features_signal;
//       features_signal.total_length = sizeof(features) / sizeof(features[0]);
//       features_signal.get_data = &raw_feature_get_data;

//       // invoke the impulse
//       EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);

//       ei_printf("run_classifier returned: %d\n", res);

//       if (res != 0)
//         return 1;

//       ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
//                 result.timing.dsp, result.timing.classification, result.timing.anomaly);

//       // print the predictions
//       ei_printf("[");
//       for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
//       {
//         ei_printf("%.5f", result.classification[ix].value);
// #if EI_CLASSIFIER_HAS_ANOMALY == 1
//         ei_printf(", ");
// #else
//         if (ix != EI_CLASSIFIER_LABEL_COUNT - 1)
//         {
//           ei_printf(", ");
//         }
// #endif
//       }
// #if EI_CLASSIFIER_HAS_ANOMALY == 1
//       printf("%.3f", result.anomaly);
// #endif
//       printf("]\n");

//       ei_sleep(500);
//     }
//   }
}