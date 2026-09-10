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

## Profiles

Profiles can limit motor current, speed, duty cycle, and power. They are applied
temporarily in controller RAM and do not overwrite the controller's stored
motor configuration. The active profile is checked periodically and reapplied
only when required.

## Requirements

- Waveshare ESP32-S3 Touch AMOLED 1.75 display running its matching VESC
  Express firmware
- A powered VESC controller publishing CAN status frames

## Configuration

Connect directly to the display and open the package's App UI tab to configure
settings and profiles.

Package fonts can be regenerated with `tools/make_fonts.ps1`. Only distribute
font assets when permitted by their license.
