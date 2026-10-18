# VESC Express

The is the codebase for the VESC Express, which is a WiFi and Bluetooth-enabled logger and IO-board. At the moment it is tested and runs on the ESP32C3, ESP32C6 and ESP32S3 but other ESP32 devices can be added.

## Toolchain

Instructions for how to set up the toolchain can be found here:
[https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html)

### Get Release 6.1.0

The instructions linked above will install the master branch of ESP-IDF. To install the stable release you can navigate to the installation directory and use the following commands:

```bash
git clone -b v6.1 --recursive https://github.com/espressif/esp-idf.git esp-idf-v6.1
cd esp-idf-v6.1/
./install.sh esp32c3 esp32c6 esp32s3 esp32p4
```

Development uses ESP-IDF 6.1.0. Note that different IDF-versions are very likely to cause compatibility issues, so it is strongly recommended to use version 6.1.0.

## Building

Set the target chip/architecture with 
```bash
idf.py set-target <target> 
```

where target is esp32c3, esp32c6, esp32s3 or esp32p4. You will need to run a fullclean or remove the build directory when changing targets.

Each hardware target uses its matching `sdkconfig.defaults.<hw_target>` profile.

Boards that need non-default flash or PSRAM settings should instead provide their own minimal `sdkconfig.defaults.<hw_target>` file next to the shared target configs in the repository root.

The defaults contain only settings exported by `idf.py save-defconfig`. After configuring the selected hardware, run this command and copy the generated `sdkconfig.defaults` to its `sdkconfig.defaults.<hw_target>` profile.

Once the toolchain is set up in the current path, the project can be built with

```bash
idf.py build
```

That will create vesc_express.bin in the build directory, which can be used with the bootloader in VESC Tool. If the ESP32c3 does not come with firmware preinstalled, the USB-port can be used for flashing firmware using the built-in bootloader. That also requires bootloader.bin and partition-table.bin which also can be found in the build directory. This can be done from VESC Tool or using idf.py.

All targets can be built with

```bash
python build_all.py
```

That will create all required firmware files under the build_output directory, with hardware names as child directories. Each hardware configuration uses a separate build directory and sdkconfig.

### Custom Hardware Targets

If you wish to build the project with custom hardware config files you should add the hardware config files to the "**main/hwconf**" directory and use the HW_NAME build flag
```bash
idf.py build -DHW_NAME="VESC Express T"
```

**Note:** If you ever change the environment variables, or if when you first start using them, you need to first run `idf.py reconfigure` before building (with the environment variables still set of course!), as the build system unfortunately can't automatically detect this change. Running `idf.py fullclean` has the same effect as this forces cmake to rebuild the build configurations.
