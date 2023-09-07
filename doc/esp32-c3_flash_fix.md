# ESP32-C3 Flash Fix

Some ESP32-C3 chips refuse to flash over USB using esptool for the first time for some reason. After a lot of trial and error, the following procedure has worked for me for using openocd. If there is an easier way to achieve the same, please let me know!

1. Install the latest openocd

```bash
git clone https://github.com/ntfreak/openocd.git openocd
cd openocd
sudo apt install libusb-1.0-0-dev libtool
./bootstrap
./configure
make -j8
sudo make install
```

2. Install udev rules

```bash
sudo mv 48-esp.rules /etc/udev/rules.d/
sudo udevadm trigger
```

3. Reset the ESP32-C3 while GPIO9 is held low
4. Use the following openocd-commands to flash a firmware after compiling it

```bash
get_idf # Get idf environment

openocd -f board/esp32c3-builtin.cfg -c "program_esp build/partition_table/partition-table.bin 0x8000 verify reset exit"

openocd -f board/esp32c3-builtin.cfg -c "program_esp build/bootloader/bootloader.bin 0 verify reset exit"

openocd -f board/esp32c3-builtin.cfg -c "program_esp build/vesc_express.bin 0x20000 verify reset exit"
```

From this point on esptool and VESC Tool should work as normal to flash firmware.
