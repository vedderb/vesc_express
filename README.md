# VESC Express

This is the codebase for the VESC Express, which is a WiFi and Bluetooth-enabled logger and IO-board. At the moment it is tested and runs on the ESP32C3, ESP32C6 and ESP32S3 but other ESP32 devices can be added.

## Toolchain

Instructions for how to set up the toolchain can be found here:
[https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html)

### Get Release 6.0.1

The instructions linked above install the latest ESP-IDF branch. To install the supported stable release you can navigate to the installation directory and use the following commands:

```bash
git clone -b v6.0.1 --recursive https://github.com/espressif/esp-idf.git esp-idf-v6.0.1
cd esp-idf-v6.0.1/
./install.sh esp32c3 esp32c6 esp32s3
```

At the moment development is done using the stable 6.0.1 release. Note that different IDF versions are very likely to cause compatibility issues, so it is strongly recommended to use version 6.0.1.

## Building

Set the target chip/architecture with 
```bash
idf.py set-target <target> 
```

where target is esp32c3, esp32c6 or esp32s3. You will need to run a fullclean or remove the build directory when changing targets.

Each normal build target uses its own shared base file `sdkconfig.defaults.<target>`.

Boards that need non-default flash or PSRAM settings can provide `sdkconfig.defaults.<hw_file>` next to the shared target configs in the repository root. Hardware defaults are applied after the shared target defaults, so they only need to contain the board-specific overrides.

For ESP32-C6 builds, use `Devkit C6 8M` for 8 MB flash modules and `Devkit C6 4M` for 4 MB flash modules.

The private JFBMS master/slave CAN bus uses standard 11-bit CAN IDs. The frame
layout is documented in [main/hwconf/jetfleet/jfbms_slave/BMS_MASTER_SLAVE_PROTOCOL.md](main/hwconf/jetfleet/jfbms_slave/BMS_MASTER_SLAVE_PROTOCOL.md).

Once the toolchain is set up in the current path, the project can be built with

```bash
idf.py build
```

That will create vesc_express.bin in the build directory, which can be used with the bootloader in VESC Tool. If the ESP32C3 does not come with firmware preinstalled, the USB-port can be used for flashing firmware using the built-in bootloader. That also requires bootloader.bin and partition-table.bin which also can be found in the build directory. This can be done from VESC Tool or using idf.py.

All targets can be built with

```bash
python build_all.py
```

That will create all required firmware files under the build_output directory, with hardware names as child directories. All target switching is handled automatically with the build_all command.

### Custom Hardware Targets

If you wish to build the project with custom hardware config files you should add the hardware config files to the "**main/hwconf**" directory and use the HW_NAME build flag
```bash
idf.py build -DHW_NAME="VESC Express T"
```

**Note:** If you ever change the environment variables, or if when you first start using them, you need to first run `idf.py reconfigure` before building (with the environment variables still set of course!), as the build system unfortunately can't automatically detect this change. Running `idf.py fullclean` has the same effect as this forces cmake to rebuild the build configurations.
