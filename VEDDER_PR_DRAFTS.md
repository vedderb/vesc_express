# Proposed PR Split for vedderb/vesc_express

This split is ordered by likely mergeability. The goal is to keep each PR reviewable on its own and avoid making the ESP-IDF v6 migration block smaller improvements.

Do not include these local-only changes in upstream PRs:

- `CMakeLists.txt`: local default `set(HW_NAME "JFBMS32v2")`. Keep this commented or leave upstream default behavior unchanged.
- Untracked Lisp scripts until they are cleaned into a separate app/script PR:
  - `main/hwconf/jetfleet/jfbms32/jfbms32_main.lisp`
  - `main/hwconf/jetfleet/jfbms_slave/jfbms_slave_main.lisp`
  - `main/hwconf/jetfleet/jfbms_slave/jfbms_slave_sim.lisp`

## PR 1: Small BMS and sleep reliability fixes

Branch: `pr/bms-lispif-reliability-fixes`

Suggested commits:

- `7c15ac48 lispif: fix uint32_t wraparound in sleep timer`
- `45672078 lispif: tear down radios before sleep, split for light vs deep`
- `a6753258 bms: fix aggregate values reply capacity`

Title:

```text
Fix BMS aggregate reply capacity and improve sleep handling
```

Body:

```markdown
This PR contains a small set of reliability fixes that are independent of new hardware targets.

Changes:
- Fixes the BMS aggregate values reply capacity so all values fit in the response.
- Fixes uint32 wraparound handling in the Lisp sleep timer path.
- Splits light/deep sleep handling and tears down radios before sleep to reduce wake/sleep edge-case failures.

Motivation:
These fixes are useful for existing hardware targets and do not depend on JetFleet-specific hardware or ESP-IDF v6 migration work.

Tested:
- Built locally for the existing VESC Express targets.
- Runtime-tested on BMS hardware through repeated sleep/wake cycles.
```

Merge chance: high. This is the best first PR because it is small, useful outside JetFleet, and does not ask Vedder to accept a new hardware family.

## PR 2: Add generic hardware CAN hooks and headless slave build toggles

Branch: `pr/hw-can-hooks-slave-toggles`

Suggested commits:

- `43f4d3a5 Add HW_CAN_PING_SCAN_ENABLED hwconf toggle`
- `fe5adf34 Add HW_CAN_NO_ACK_MODE hwconf toggle`
- `75d67a23 Add hardware CAN filter and RX hooks`
- `b04503ff Add HW_IS_SLAVE compile-time mode for headless variants`

Title:

```text
Add hardware-level CAN hooks and optional slave-mode toggles
```

Body:

```markdown
This PR adds a few optional hardware configuration hooks used by headless and private-CAN hardware variants.

Changes:
- Adds `HW_CAN_PING_SCAN_ENABLED` to let hardware configs opt out of CAN ping scans.
- Adds `HW_CAN_NO_ACK_MODE` for hardware that needs CAN no-ack mode.
- Adds optional hardware CAN filter/RX hooks.
- Adds `HW_IS_SLAVE` for headless variants that should not initialize the full normal application role.

Motivation:
These are compile-time hooks with default behavior unchanged for existing hardware. They make it possible to support simpler/headless CAN devices without adding board-specific conditionals to common code.

Compatibility:
Existing hardware targets keep the previous behavior unless they define the new macros.

Tested:
- Built locally with existing default targets.
- Tested with the JFBMS slave hardware variant that depends on these hooks.
```

Merge chance: high to medium. It is generic and small, but Vedder may ask for naming or macro style changes.

## PR 3: Add JFBMS32 hardware target

Branch: `pr/jfbms32-hardware-target`

Suggested commit:

- `2f6ea9d4 Add JFBMS32 hardware target`

Title:

```text
Add JetFleet JFBMS32 hardware target
```

Body:

```markdown
This PR adds the JetFleet JFBMS32 hardware target for VESC Express.

Changes:
- Adds JFBMS32 v1 and v2 hardware definitions.
- Adds BQ769x2 support definitions used by the target.
- Adds JFBMS32 configuration parser, defaults, XML metadata, and VESC Tool settings.
- Adds the small command hook needed by this target.

Hardware summary:
- ESP32-based BMS controller.
- Dual BQ769x2 support.
- Charge/output voltage measurement.
- Charge FET, communication enable, power switch, shutdown, CAN, I2C, and temperature support.

Motivation:
This makes the JFBMS32 target buildable from the upstream tree without carrying a private hardware patch.

Tested:
- Built locally as `HW_NAME=JFBMS32v1`.
- Built locally as `HW_NAME=JFBMS32v2`.
- Runtime-tested on JFBMS32 hardware.
```

Merge chance: medium. It is isolated but large. Keep it independent from the slave/master expansion and from ESP-IDF v6.

## PR 4: Add JFBMS slave hardware target and protocol documentation

Branch: `pr/jfbms-slave-hardware-target`

Suggested commit:

- `e2a5723b Add JFBMS slave hardware variant`

Title:

```text
Add JetFleet JFBMS slave hardware target
```

Body:

```markdown
This PR adds the JetFleet JFBMS slave hardware target and documents the private master/slave BMS CAN protocol.

Changes:
- Adds the `JFBMS_SLAVE` hardware target.
- Adds BQ769x2 definitions for the slave board.
- Adds config defaults, parser, XML metadata, and VESC Tool settings.
- Documents the standard 11-bit CAN protocol used between JFBMS32 master and slave boards.

Protocol summary:
- Slaves broadcast cell voltages, temperatures, status, and balancing state.
- The master sends balance commands.
- Status frames include per-IC cell counts and fault flags.
- A watchdog disables slave balancing if commands stop.

Motivation:
This target supports pack expansion beyond the cells directly handled by the master BMS controller while keeping the protocol documented and reviewable.

Tested:
- Built locally as `HW_NAME=JFBMS_SLAVE`.
- Runtime-tested on slave BMS hardware with JFBMS32/JF Link master-side receivers.
```

Merge chance: medium to low. It is self-contained, but Vedder may prefer to merge the JFBMS32 master first and review the private CAN protocol separately.

## PR 5: Add JF Link hardware target for aggregated JFBMS data

Branch: `pr/jf-link-hardware-target`

Suggested commit:

- `2dd2e196 Add JF Link hardware target`

Title:

```text
Add JetFleet JF Link hardware target
```

Body:

```markdown
This PR adds the JetFleet JF Link hardware target.

Changes:
- Adds the JF Link hardware definition and configuration files.
- Adds logic for receiving JFBMS slave data and exposing aggregated BMS values.
- Adds balancing coordination for multiple JFBMS slave boards.
- Extends BMS value handling for this aggregated-cell use case.

Motivation:
JF Link provides a bridge/aggregator target for systems using multiple JFBMS slave boards over the documented private CAN protocol.

Tested:
- Built locally as `HW_NAME=JF Link`.
- Runtime-tested with JFBMS slave boards broadcasting voltage, temperature, status, and balance state.
```

Merge chance: medium to low. This is useful after PR 4, but it depends on Vedder accepting the broader JetFleet/JFBMS architecture.

## PR 6: Add ESP32-C6 4 MB/8 MB devkit split and NimBLE backend

Branch: `pr/esp32c6-devkits-nimble`

Suggested commits:

- `b59e942c ble: add NimBLE backend for ESP32-C6`
- `d86df2da Add C6 4M and 8M devkit configs`

Title:

```text
Add ESP32-C6 NimBLE support and split C6 devkit flash variants
```

Body:

```markdown
This PR improves ESP32-C6 support.

Changes:
- Adds a NimBLE BLE backend for ESP32-C6.
- Adds separate ESP32-C6 devkit hardware targets for 4 MB and 8 MB flash variants.
- Adds hardware-specific sdkconfig defaults for the 4 MB C6 variant.
- Updates the README to describe the C6 target choices.

Motivation:
ESP32-C6 uses a different BLE stack path than the earlier supported targets. Splitting 4 MB and 8 MB devkit configs also avoids forcing one flash layout onto both module variants.

Tested:
- Built locally for ESP32-C6 4 MB and 8 MB devkit targets.
- Tested BLE communication on ESP32-C6 hardware.
```

Merge chance: medium. This is not JetFleet-specific, but NimBLE is a large addition. If Vedder hesitates, split this into two PRs: C6 4M/8M target split first, NimBLE backend second.

## PR 7: Migrate vesc_express to ESP-IDF v6.0.1

Branch: `pr/esp-idf-v6-standalone`

Suggested commit:

- `87f6872e Migrate project to ESP-IDF v6.0.1`

Title:

```text
Migrate project to ESP-IDF v6.0.1
```

Body:

```markdown
This PR migrates VESC Express from the current ESP-IDF 5.x baseline to ESP-IDF v6.0.1.

Changes:
- Updates the documented toolchain version to ESP-IDF v6.0.1.
- Updates component dependencies and sdkconfig defaults.
- Adapts ADC, I2C, CAN/TWAI, BLE, WiFi, USB, display, touch, and driver code for IDF v6 APIs.
- Adds I2C compatibility wrappers used by existing hardware code.
- Updates bootloader component handling for IDF v6.

Motivation:
ESP-IDF v6 is the current stable baseline for ongoing development in this branch. This PR keeps the migration standalone so it can be reviewed and accepted or rejected independently from new hardware targets.

Compatibility:
This is intentionally not bundled with JFBMS/JF Link hardware support. If upstream prefers to stay on ESP-IDF 5.x for now, the earlier PRs can still be reviewed separately.

Tested:
- Built locally with ESP-IDF v6.0.1.
- Built the existing ESP32-C3, ESP32-C6, and ESP32-S3 targets.
- Runtime-tested on available hardware after migration.
```

Merge chance: low to medium. This should stay standalone because Vedder may decide not to move upstream to ESP-IDF v6 yet.

## Recommended submission order

1. PR 1: small BMS/Lisp sleep reliability fixes.
2. PR 2: generic CAN/headless hardware hooks.
3. PR 3: JFBMS32 hardware target.
4. PR 4: JFBMS slave target and protocol docs.
5. PR 5: JF Link target.
6. PR 6: ESP32-C6 support, optionally split into devkit flash variants and NimBLE.
7. PR 7: ESP-IDF v6 migration, standalone.

If you want the highest chance of getting something merged quickly, send only PR 1 and PR 2 first. Wait for review feedback before opening the large JetFleet hardware PRs.
