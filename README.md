# Person Detection on Arducam Pico4ML

Runs exported Edge Impulse C++ library on RP2040-based Pico4ML board. See the documentation at [Running your impulse locally](https://docs.edgeimpulse.com/docs/running-your-impulse-locally-1). This repository is based off of the [Arducam Pico4ML Magic Wand Example](https://github.com/ArduCAM/Pico4ML-Magic-Wand/).

Note that the code in the *test/* folder are standalone projects for prototyping. They are not used in this program.

## TODO

Note that the LCD does not work at this time, so it is commented out (through a #define in *source/main.cpp*). I don't think I'm filling the buffer or calling `ST7735_DrawImage()` correctly, so this is something to dig into. I just don't have the time right now, and it's ultimately for debugging (as there's no LCD on the custom person detection board).

Also, you may want to look at optimizing the program for dual-core operation. For example, have the image capture and resize process run on one core, send the data in a FIFO to the second core, which just runs inference.

## Requirements

### Hardware

* [Arducam Pico4ML Board](https://www.arducam.com/pico4ml-an-rp2040-based-platform-for-tiny-machine-learning/)

### Software

* Raspberry Pi Pico SDK [Install instructions](https://github.com/pimoroni/pimoroni-pico/blob/main/setting-up-the-pico-sdk.md)
* Make sure that the environment variable `PICO_SDK_PATH` is set to the location of the Pico SDK on your computer

## Build Application

Download the *C++ SDK library* from Edge Impulse. Unzip it into *source/edge-impulse* (replace any files/folders). From the top-level directory of this repository, call the following:

```bash
mkdir build
cd build
cmake ..
make -j4
```

Put the Raspberry Pi Pico into [bootloader mode](https://helloraspberrypi.blogspot.com/2021/02/raspberry-pi-pico-enter-bootloader-mode.html). Copy the *app.uf2* file from *build/* to the Raspberry Pi Pico drive.

Connect to the Pico with a serial program (e.g. PuTTY, screen) with: 115200 baud, 8N1. You should see inference results being written over the serial connection.

## License

Unless otherwise specified, code in this repository is licensed under the APACHE 2.0 open source license.

Copyright 2023 EdgeImpulse, Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.