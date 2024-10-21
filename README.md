# VESC Express

The is the codebase for the VESC Express, which is a WiFi and Bluetooth-enabled logger and IO-board. At the moment it is tested and runs on the ESP32C3, but it might work on other ESP32-devices too.

## Toolchain

Instructions for how to set up the toolchain can be found here:
[https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html)

**Note**  
ESP-IDF version 5.2 or later is required for building this project.

### Get Release 5.2.2

The instructions linked above will install the master branch of ESP-IDF. To install the stable release you can navigate to the installation directory and use the following commands:

```bash
git clone -b v5.2.2 --recursive https://github.com/espressif/esp-idf.git esp-idf-v5.2.2
cd esp-idf-v5.2.2/
./install.sh esp32c3
```

At the moment development is done using the stable 5.2.2-release.

## Building

Once the toolchain is set up in the current path, the project can be built with

```bash
idf.py build
```

That will create vesc_express.bin in the build directory, which can be used with the bootloader in VESC Tool. If the ESP32c3 does not come with firmware preinstalled, the USB-port can be used for flashing firmware using the built-in bootloader. That also requires bootloader.bin and partition-table.bin which also can be found in the build directory. This can be done from VESC Tool or using idf.py.

### Custom Hardware Targets

If you wish to build the project with custom hardware config files you have two options:
1. Add the hardware files to the "**main/hwconf**" directory.
2. Set the `HW_SRC` and `HW_HEADER` environment variables.

For option 1. you could for instance add you're two files `hw_my_device.c` and `hw_my_device.h` into "**main/hw_conf**", and then edit the `HW_SOURCE` and `HW_HEADER` macro definitions in "**main/conf_general.h**" to the names of your new files. This method is ideal if you may want to contribute back these hardware configurations to the vesc_express repository!

Option 2. is instead better if you have hardware configuration files in a directory which is outside the vesc_express source tree. You would then, for instance, set the environment variables `HW_SRC=/my/path/to/hw_my_device.c` and `HW_HEADER=/my/path/to/hw_my_device.h` when running `idf.py build`.

**Note:** If you ever change the environment variables, or if when you first start using them, you need to first run `idf.py reconfigure` before building (with the environment variables still set of course!), as the build system unfortunately can't automatically detect this change. Running `idf.py fullclean` has the same effect as this forces cmake to rebuild the build configurations.
