# VESC Express

The is the codebase for the VESC Express, which is a WiFi and Bluetooth-enabled logger and IO-board. At the moment it is tested and runs on the ESP32C3, but it might work on other ESP32-devices too.

## Toolchain

Instructions for how to set up the toolchain can be found here:
[https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html)

**Note**  
ESP-IDF version 5.0 or later is required for building this project.

## Compiling

One the toolchain is set up and in the current path, the project can be build like this

```bash
idf.py build
```
