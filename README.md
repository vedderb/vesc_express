# VESC Express

The is the codebase for the VESC Express, which is a WiFi and Bluetooth-enabled logger and IO-board. At the moment it is tested and runs on the ESP32C3, but it might work on other ESP32-devices too.

## Toolchain

Instructions for how to set up the toolchain can be found here:
[https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html)

**Note**  
ESP-IDF version 5.0 or later is required for building this project.

### Get Release 5.0.2

The instructions linked above will install the master branch of ESP-IDF. To install the stable release you can navigate to the installation directory and use the following commands:

```bash
git clone -b v5.0.2 --recursive https://github.com/espressif/esp-idf.git esp-idf-v5.0.2
cd esp-idf-v5.0.2/
./install.sh esp32c3 esp32 esp32s3
source export.sh
```

At the moment development is done using the stable 5.0.2-release.

Additionally you may want to add export.sh to your .bashrc file.
```echo "source /path/to/esp-idf-v5.0.2/export.sh" >> ~/.bashrc```

## Compiling

Once the toolchain is set up in the current path, the project can be built with

```bash
./build.sh
```

That will create vesc_express.$target.bin in the bin directory, which can be used with the bootloader in VESC Tool. If the ESP32, ESP32c3 or ESP32s3 does not come with firmware preinstalled, the USB-port can be used for flashing firmware using the built-in bootloader. That also requires bootloader.bin and partition-table.bin which also can be found in the build directory. This can be done from VESC Tool or using idf.py.
