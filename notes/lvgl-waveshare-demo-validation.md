# LVGL Waveshare 1.75 dashboard validation

Date: 2026-08-17

## Delivered runtime

- New package: `vesc_pkg/lvgl_waveshare_175_demo`
- Package output: `lvgl_waveshare_175_demo.vescpkg`
- Target: Waveshare ESP32-S3 Touch AMOLED 1.75, 466 x 466
- Physical UI: RFP200-derived Main page using the Eurostile display font and
  original battery, controller, and motor icons
- Live behavior: the package auto-selects the first controller with a fresh
  CAN status-1 frame and drives speed, battery, controller temperature, motor
  temperature, power, and all three arcs from VESC Express data
- Stale behavior: each required CAN status group expires independently after
  one second; unavailable values show dashes and their arcs return to zero
- Drive-mode behavior: the physical dashboard shows the Eco, Sport, or Race
  artwork only after the selected temporary motor-controller profile has been
  acknowledged by the live controller
- Administration surface: a non-fullscreen QML App UI configures brightness,
  speed and power units, and three saved temporary controller profiles
- Restart behavior: the physical dashboard is created before profile recovery
  starts, preventing a profile initialization failure from leaving the native
  `Hello LVGL` fallback visible

## Firmware validation

Build command:

```powershell
. C:\esp\v5.5.4\esp-idf\export.ps1
idf.py -B build-demo-waveshare build
```

Result: PASS.

- Image: `build-demo-waveshare/vesc_express_lvgl.bin`
- Size: 1,533,344 bytes (`0x1765a0`)
- App partition free: 63%
- SHA-256:
  `7F4AF009C23E7C51508D1A5AF4659F46B16E647D09D7815CF622830B777B0337`

The complete ESP32-S3 image was flashed to COM29. Esptool identified the
expected ESP32-S3, 8 MB PSRAM, MAC `80:b5:4e:da:66:e8`, and verified every
written region by SHA digest.

## Package validation

The live dashboard package was built with VESC Tool 7.00 from `pkgdesc.qml`.

- Size: 30,058 bytes
- SHA-256:
  `D7D92B2010ACDC4A06B6320B2B673878540AC937523545BE21C3097626EBA3C1`
- QML payload: `ui.qml` configuration App UI
- LispBM payload: `main.lisp`, four LVGL binary fonts, and six LVIM image
  resources

VESC Tool 7.00 parsed the current LispBM payload, found the QML payload, and
accepted only `custom:waveshare amoled 1.75` in the package compatibility
tests. An offscreen QML load also completed without runtime errors.

The user installed the `.vescpkg` through VESC Tool and confirmed that the
package works on the physical display. Compressed-font decoding was then
enabled in firmware after all labels initially rendered blank. The user
confirmed that the corrected fonts render visibly. The font subsets now also
include the minus glyph used by stale markers and signed regenerative power.

The package fonts and central speed/power layout now reproduce RFP200's
`speedversion2` contract: JM100 speed, JM40 power, JM30 units/battery, and
JM20 temperatures. The font line-height and baseline metrics, label widths,
alignment anchors, and x/y offsets match `ui_screen_mainui.c`.

The QML App UI stores display brightness, km/h or mph, watts or battery amps,
and Eco/Sport/Race limits in LispBM EEPROM. Profile application uses temporary
motor-controller configuration commands only, so it does not replace the
controller's stored configuration. The guardian reapplies the selected saved
profile after startup or controller reconnection and checks the temporary
configuration fingerprint every 15 seconds before deciding whether another
write is required.

VESC Tool 7.00 uploaded the current non-reduced packed Lisp/import payload to
COM29 and logged `LispBM upload OK!`. A post-upload firmware query exited 0 and
returned V7.00, `Waveshare AMOLED 1.75`, UUID
`80 B5 4E DA 66 E8 00 00 00 00 00 00`.

The `--bridgeAppData` runtime check received the package heartbeat continuously:

```text
lvgl-live id=-1 s1=0 s4=0 s5=0
```

This proves the uploaded Lisp update loop is running. It also proves that no
controller was broadcasting fresh status 1, 4, or 5 frames during this bench
capture, so live nonzero telemetry remains a controller-on-CAN bench test.

The current 30,058-byte package containing the QML settings page and profile
guardian has been built and QML/package-compiler validated but has not been
installed on the hardware. Hardware behavior for automatic profile restoration
and periodic alignment therefore remains unverified.

## Live telemetry mapping

- Speed: `canget-rpm`, converted from electrical RPM using package-local motor
  pole-pair, gear-ratio, and wheel-diameter calibration
- Battery: fresh `get-bms-val 'bms-soc` first; otherwise `canget-vin` scaled by
  the package-local battery voltage range
- Controller temperature: `canget-temp-fet` from fresh status 4
- Motor temperature: `canget-temp-motor` from fresh status 4; the no-sensor
  negative sentinel is rejected
- Power: signed `canget-vin * canget-current-in` from fresh status 5 and 4
- Controller discovery: first entry from `can-list-devs` whose status-1 age is
  under one second; no active 254-ID `can-scan` occurs in the UI loop

The package defaults match RFP200's fallback calibration: 28 motor poles,
direct drive, 500 mm wheel, and a 13S lithium voltage range. They can be edited
in `main.lisp` and repackaged without reflashing the generic Express firmware.

## Recovery finding

A separate diagnostic attempt using the raw CLI path
`--uploadLisp main.lisp --reduceLisp` caused the physical screen and VESC
communications to stop. Erasing only the 512 KB `lisp` partition at
`0x820000` restored the known-good `Hello LVGL` screen, and both VESC Tool 7.00
and 7.01 firmware queries returned success afterward.

The actual `.vescpkg` installed through VESC Tool works. Therefore the reduced
raw-Lisp CLI route is not a supported installation method for this resource
package. The later non-reduced `--uploadLisp main.lisp` path uses the same
import packer as package construction and completed successfully; keep
`--reduceLisp` disabled for these binary resources.

## Source checks

- Handle registry uses opaque generation-checked integers; no raw LVGL
  pointers cross into LispBM.
- Package-owned screens, fonts, images, and copied resource buffers are
  released on Lisp restart.
- LVGL access is serialized through the firmware runtime lock.
- `git diff --check`: PASS (line-ending notices only; no whitespace errors).
