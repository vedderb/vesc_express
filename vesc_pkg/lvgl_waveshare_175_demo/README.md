# LVGL Waveshare 1.75 Dashboard

This package recreates the essential RFP200 Main UI on the 466 x 466
Waveshare ESP32-S3 Touch AMOLED 1.75 target. It uses LVGL widgets, the original
Eurostile-derived display font, and the battery, controller, motor, Eco, and
Race icon artwork from the user's RFP200 project.

## BMS scope

This package does not provide a BMS function. It has no BMS page, BMS scan or
connection workflow, cell telemetry, balancing controls, or BMS configuration.
Use VESC Tool's standard BMS pages for those features.

The Main dashboard's battery gauge is display-only. It can passively use a
standard VESC BMS state-of-charge value that is already present on the VESC
network; otherwise it estimates charge from controller input voltage. Reading
that existing value does not add BMS management or configuration to this
package.

Every displayed value is live. The package automatically selects the first
VESC controller with a fresh CAN status-1 frame, then maps the available VESC
Express data as follows:

- speed: status-1 electrical RPM, displayed in the selected km/h or mph unit
- battery gauge: existing standard VESC BMS SOC when available, otherwise an
  estimate from status-5 input voltage; this is not a BMS control function
- controller and motor temperatures: status 4
- selected power value: signed input watts or signed battery/input amps

Each CAN frame group has an independent one-second stale timeout. Missing data
is shown with dashes and its arc returns to zero instead of retaining an old or
synthetic value. The running loop emits one `lvgl-live` diagnostic heartbeat
per second through LispBM app data.

The package also owns three saved temporary motor-controller profiles: Eco,
Sport, and Race. The App UI can tune motor-current scale, vehicle speed,
duty-cycle, and watt limits for each profile. Save & Apply asks an
Express-resident guardian to forward `COMM_SET_MCCONF_TEMP_SETUP` to the live
controller selected by the display. It writes controller RAM only
(`store = false`) and never replaces the controller's stored motor
configuration.

The matching RFP200 profile icon appears on the physical dashboard only after
the guardian receives the controller acknowledgement. Eco uses the green Eco
artwork, Sport uses the white Race artwork, and Race uses the red Race artwork,
matching the original RFP200 positioning and recoloring. The guardian hides the
icon when controller synchronization becomes stale, reapplies the saved active
profile when the controller returns, and restores the icon after a new
acknowledgement.

Every 15 seconds the guardian reads `COMM_GET_MCCONF_TEMP` and compares a CRC32
fingerprint with the configuration captured after the acknowledged write. It
reapplies only when the fingerprint differs; the periodic check does not write
EEPROM or repeatedly rewrite an already-aligned controller.

The VESC Tool **App UI** tab configures AMOLED brightness, speed unit, power
unit, and the three temporary profiles. Display settings and profile values are
stored in LispBM EEPROM and survive a display restart. Battery amps uses VESC
status-4 input current rather than motor phase current. Once a profile has been
successfully activated, Express restores it automatically after either the
display package or the motor controller restarts.

The physical dashboard is loaded before profile EEPROM and command-guardian
initialization. Mode artwork is loaded on demand. A profile subsystem error
therefore cannot leave the display on the native `Hello LVGL` fallback.

`main.lisp` contains package-local speed and voltage calibration constants.
Their defaults match the original RFP200 fallback (28 motor poles, direct
drive, 500 mm wheel, 13S lithium pack). An existing standard VESC BMS SOC value
takes priority for the display-only battery gauge; otherwise voltage scaling is
used. These constants can be changed in the package without reflashing the
generic VESC Express firmware.

## Requirements

- VESC Express hardware name: `Waveshare AMOLED 1.75`
- Firmware with the handle-based LispBM-to-LVGL bridge in this checkout
- LVGL 9.5 with Label, Arc, Image, RGB565-swapped, and RGB565A8 support

## Install

Build `lvgl_waveshare_175_demo.vescpkg`, then install it from the VESC Packages
page in VESC Tool. Open the package's App UI tab while connected directly to
the display to read or apply its settings and profiles. The motor controller
must be powered and publishing fresh status frames for profile application.

## Resource format

Fonts are LVGL binary fonts generated with `lv_font_conv`. Images use the
small package-local `LVIM` container documented in `tools/make_assets.py`; the
firmware copies every resource and exposes only opaque handles to LispBM.

Regenerate the subset fonts from the original RFP200 typeface with
`tools/make_fonts.ps1`. The script also restores the canonical RFP200 line
height and baseline metrics so content-sized labels align exactly like the
compiled RFP200 fonts.

The icon and UI artwork remains derived from the user's RFP200 project. The
Eurostile font remains subject to the license under which it is installed on
the build machine; do not redistribute this package without confirming that
license.
