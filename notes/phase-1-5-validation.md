# VESC Express LVGL phases 1-5 validation

Date: 2026-08-17

## Source baseline

- Repository: `https://github.com/vedderb/vesc_express.git`
- Branch: `main`
- Commit: `38810153bc06923f828002be660bc7e9d54921c6`
- Local `HEAD` matched `origin/main` before implementation.

## Locked dependencies

- ESP-IDF: 5.5.4 (`>=5.5.4,<5.6.0` component constraint)
- LVGL: 9.5.0
- Espressif CO5300 component: 2.1.0

The shared `sdkconfig.defaults.lvgl` overlay enables the FreeRTOS LVGL OS
port, RGB565 and RGB565-swapped software drawing, the label widget, and
Montserrat 14. It disables themes, layouts, demos, examples, and unused draw
formats/fonts required by neither the phase-5 smoke screen nor the display
backend.

## Implemented scope

- Shared, mutex-protected native display backend.
- AXS15231 migrated to the backend while retaining the existing LispBM
  `disp-render`, `disp-clear`, and reset callback path.
- Official Espressif CO5300 QSPI driver integration.
- Waveshare ESP32-S3-Touch-AMOLED-1.75 hardware target with AXP2101 display
  power initialization and the validated pin map.
- Owned LVGL task, tick source, lock helpers, lifecycle hooks, and memory
  statistics.
- 466x466 RGB565-swapped LVGL display with a 466x40-line internal DMA partial
  buffer and a firmware-owned `Hello LVGL` smoke screen.
- An intentionally empty phase-7 LispBM/LVGL extension seam; raw LVGL pointers
  are not exposed.

## Build validation

Existing ESP32-C3 target regression build:

```powershell
. C:\esp\v5.5.4\esp-idf\export.ps1
idf.py -B build-c3-lvgl-2 '-DSDKCONFIG=sdkconfig_test' build
```

Result: PASS. The generated application image was `0x16ad60` bytes in the
`0x190000` app partition, leaving 9% free.

Fresh Waveshare target build after the final LVGL configuration update:

```powershell
. C:\esp\v5.5.4\esp-idf\export.ps1
idf.py -B build-final-waveshare '-DHW_NAME=Waveshare AMOLED 1.75' '-DSDKCONFIG=sdkconfig_final' build
```

Result: PASS. The build selected `Waveshare AMOLED 1.75 on esp32s3_n16r8` and
produced an ESP32-S3 image of `0x16fc60` bytes in the `0x400000` app partition,
leaving 64% free. The generated configuration confirmed:

- `CONFIG_LV_CONF_MINIMAL=y`
- `CONFIG_LV_OS_FREERTOS=y`
- `CONFIG_LV_COLOR_DEPTH_16=y`
- `CONFIG_LV_DRAW_SW_SUPPORT_RGB565=y`
- `CONFIG_LV_DRAW_SW_SUPPORT_RGB565_SWAPPED=y`
- `CONFIG_LV_FONT_MONTSERRAT_14=y`
- `CONFIG_LV_FONT_DEFAULT_MONTSERRAT_14=y`
- UNSCII 8, themes, examples, and demos disabled

`git diff --check` passed. Compiler warnings came from unchanged upstream or
managed dependency code (deprecated Bluetooth APIs, unused existing
functions, and the managed SPI NAND component).

## Remaining proof boundary

No board was flashed during this milestone. Build and configuration proof are
complete; AMOLED power-up, panel alignment, visible `Hello LVGL`, and retained
Lisp rendering still require hardware bench validation. Touch input and the
handle-based LispBM LVGL bridge remain later phases in `notes/plan.md`.
