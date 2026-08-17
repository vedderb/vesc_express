# LVGL Waveshare 1.75 Dashboard

An LVGL dashboard package for the 466 x 466 Waveshare ESP32-S3 Touch AMOLED
1.75 display.

## Features

- Live speed, battery gauge, controller temperature, motor temperature, and
  power from the selected VESC controller
- Configurable km/h or mph and watts or battery amps
- Adjustable AMOLED brightness
- Saved Eco, Sport, and Race temporary controller profiles
- Automatic profile restoration after display or controller restart
- One-second telemetry timeout so stale values are not displayed as current

## BMS

This package has no BMS functions. It does not scan, connect, configure,
balance, or display BMS cell data.

The battery gauge can read an existing standard VESC BMS state-of-charge value
from the network. If none is available, it estimates charge from controller
input voltage. This is display-only behavior.

## Profiles

Profiles can limit motor current, speed, duty cycle, and power. They are applied
temporarily in controller RAM and do not overwrite the controller's stored
motor configuration. The active profile is checked periodically and reapplied
only when required.

## Requirements

- VESC Express hardware: `Waveshare AMOLED 1.75`
- Firmware with the LispBM-to-LVGL bridge
- A powered VESC controller publishing CAN status frames

## Install

Install `lvgl_waveshare_175_demo.vescpkg` from the VESC Packages page in VESC
Tool. Connect directly to the display to configure settings and profiles from
the package's App UI tab.

Package fonts can be regenerated with `tools/make_fonts.ps1`. Only distribute
font assets when permitted by their license.
