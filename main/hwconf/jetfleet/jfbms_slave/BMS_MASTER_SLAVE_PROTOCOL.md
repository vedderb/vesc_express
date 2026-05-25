# BMS Master-Slave CAN Protocol Specification

## Overview

11-bit standard CAN protocol for JFBMS32 master to communicate with slave BMS boards, enabling cell count expansion beyond 32 cells.

**CAN Bus:** 500 kbps

**Key Design Decisions:**
- Uses 11-bit standard CAN IDs (not 29-bit extended like VESC protocol)
- Slaves broadcast data periodically (no polling)
- Master sends only balance commands
- Only sends cell voltage messages needed for configured cells (optimizes CAN bus load)
- Status message includes cells_ic1 and cells_ic2 so master knows per-IC configuration
- Data format matches BQ76952 native format for easy integration

---

## System Architecture

```
┌─────────────────┐     CAN Bus (11-bit IDs)     ┌─────────────────┐
│   MASTER        │◄──────────────────────────────│   SLAVE 1       │
│   (JFBMS32)     │                               │   (Dual BQ76952)│
│                 │                               │   Cells 1-32    │
│ - FETs          │     ┌─────────────────┐       │   4 Temps       │
│ - Current sense │◄────│   SLAVE 2       │       └─────────────────┘
│ - Balancing     │     │   Cells 33-64   │
│   decisions     │     │   4 Temps       │
│ - SOC/SOH       │     └─────────────────┘
└─────────────────┘
```

### Master (JFBMS32)
- Has FETs (charge/discharge control)
- Has current sensor
- Makes all balancing decisions
- Aggregates cell data from all slaves
- Can work standalone (32 cells) or with slaves

### Slaves (Dual BQ76952 Boards)
- Dual BQ76952 chips (up to 32 cells, 16 per chip)
- Measure: cell voltages, 4 temperatures (BQ1 internal, BQ1 TS1, BQ2 internal, BQ2 TS1)
- Execute: balance commands from master
- NO: FETs, current sensor, sleep/shutdown
- Address: Pre-configured (not auto-discovery)

**Note:** BQ76952 hardware protections are disabled. All protection logic is implemented in the master's LispBM script.

---

## CAN ID Structure (11-bit)

```
Bit:  10  9  8  7 │ 6  5  4 │ 3  2  1  0
     ─────────────┼─────────┼───────────
      Message     │ SubType │ Slave ID
      Type (4b)   │ (3b)    │ (4b)
```

### Slave ID (Bits 3-0)
| Value | Meaning |
|-------|---------|
| 0x0   | Reserved (not used) |
| 0x1-0x8 | Slave address 1-8 |
| 0x9-0xF | Reserved |

### Message Types (Bits 10-7)

| Type | Direction | Description |
|------|-----------|-------------|
| 0x0  | Slave→Master | Cell voltages 1-4 |
| 0x1  | Slave→Master | Cell voltages 5-8 |
| 0x2  | Slave→Master | Cell voltages 9-12 |
| 0x3  | Slave→Master | Cell voltages 13-16 |
| 0x4  | Slave→Master | Cell voltages 17-20 |
| 0x5  | Slave→Master | Cell voltages 21-24 |
| 0x6  | Slave→Master | Cell voltages 25-28 |
| 0x7  | Slave→Master | Cell voltages 29-32 |
| 0x8  | Slave→Master | Temperatures |
| 0x9  | Slave→Master | Status (balance state + faults + cells per IC) |
| 0xA  | Master→Slave | Balance command + Buzzer beep code |
| 0xB-0xF | - | Reserved |

---

## Message Formats

### Cell Voltages (Types 0x0-0x7)

**CAN ID:** `(type << 7) | slave_id`

Each message carries 4 cell voltages:

```
Byte:   0      1      2      3      4      5      6      7
     ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
     │Cell 1│Cell 1│Cell 2│Cell 2│Cell 3│Cell 3│Cell 4│Cell 4│
     │[7:0] │[15:8]│[7:0] │[15:8]│[7:0] │[15:8]│[7:0] │[15:8]│
     └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
```

**Encoding:**
- 16-bit unsigned, little-endian
- Resolution: 1mV per bit (matches BQ76952)
- Range: 0-65535 mV (0V to 65.535V)

**Special Values:**
| Value | Meaning |
|-------|---------|
| 0x0000 | Cell position not populated (no cell connected) |
| 0xFFFF | BQ chip failed to initialize (all cells on that chip) |

**Cell Position Mapping:**
- Cells 1-16: BQ1 chip (messages sent only if cells configured)
- Cells 17-32: BQ2 chip (messages sent only if cells_ic2 > 0)

Master infers cell configuration from non-zero voltages. Example with 5s on BQ1, 5s on BQ2:
```
Cells:  [3500, 3501, 3499, 3500, 3502, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         3498, 3500, 3499, 3501, 3500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
```

**Example:** Cell at 3.500V
```c
// Sender (slave):
uint16_t voltage_mv = 3500;
data[0] = voltage_mv & 0xFF;        // 0xAC (low byte)
data[1] = (voltage_mv >> 8) & 0xFF; // 0x0D (high byte)

// Receiver (master):
uint16_t voltage_mv = data[0] | (data[1] << 8);  // = 3500 mV
float voltage_v = voltage_mv / 1000.0f;          // = 3.500 V
```

---

### Temperatures (Type 0x8)

**CAN ID:** `0x400 | slave_id`

```
Byte:   0      1      2      3      4      5      6      7
     ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
     │    T_BQ1    │   T_BQ1_TS1 │    T_BQ2    │   T_BQ2_TS1 │
     │    int16    │    int16    │    int16    │    int16    │
     └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
```

**DLC:** 8 bytes

**Temperature Sensors (4 total, dual BQ76952):**
| Bytes | Field | Description |
|-------|-------|-------------|
| 0-1 | T_BQ1 | BQ1 internal die temperature |
| 2-3 | T_BQ1_TS1 | External NTC on BQ1 TS1 pin |
| 4-5 | T_BQ2 | BQ2 internal die temperature (0x7FFF if single-chip) |
| 6-7 | T_BQ2_TS1 | External NTC on BQ2 TS1 pin (0x7FFF if single-chip) |

**Encoding:**
- int16, little-endian, 0.1°C resolution
- Value in units of 0.1°C (e.g., 253 = 25.3°C, -105 = -10.5°C)
- Value 0x7FFF (32767) = sensor not present or invalid

**Example:** Temperature 25.3°C
```c
int16_t temp = 253;  // 25.3°C × 10
data[0] = temp & 0xFF;         // 0xFD
data[1] = (temp >> 8) & 0xFF;  // 0x00
```

---

### Status (Type 0x9)

**CAN ID:** `0x480 | slave_id`

```
Byte:   0      1      2      3      4      5      6
     ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┐
     │BalMsk│BalMsk│BalMsk│BalMsk│Faults│IC1Cnt│IC2Cnt│
     │ [7:0]│[15:8]│[23:16]│[31:24]│      │      │      │
     └──────┴──────┴──────┴──────┴──────┴──────┴──────┘
```

**DLC:** 7 bytes (1 byte reserved for future use)

**Fields:**
| Bytes | Field | Description |
|-------|-------|-------------|
| 0-3 | BalanceMask | 32-bit bitmap, bit N = cell N is currently balancing |
| 4 | Faults | Fault flags (see below) |
| 5 | CellsIC1 | Number of cells on BQ1 (0-16) |
| 6 | CellsIC2 | Number of cells on BQ2 (0-16, 0 = single chip) |

**Fault/Flags Byte (Byte 4):**
| Bit | Meaning |
|-----|---------|
| 0 | BQ1 initialization failed |
| 1 | BQ2 initialization failed |
| 2 | Voltage-settled flag (1 = balance FETs off >= 2s, voltages are accurate) |
| 3-7 | Reserved (set to 0) |

**CellsIC1 / CellsIC2 (Bytes 5-6):**
Tells the master the exact cell configuration per BQ76952 chip. This is critical for:
- Displaying only actual cells in VESC Tool (no zeros for unpopulated positions)
- Building the correct balance mask (bits 0-15 = IC1, bits 16-31 = IC2)
- Applying per-IC balance channel limits and adjacent-cell rules

This message confirms which cells are actually balancing, allowing master to verify balance commands were applied. The fault bits indicate if slave failed to initialize a BQ76952 chip at startup.

**Backwards Compatibility:** Master accepts old 6-byte format (single CellCount byte) and interprets it as: ic1 = min(16, count), ic2 = max(0, count - 16). Master also accepts 5-byte format (no cell count) and assumes 16+16.

**Note:** Single-chip slaves (no BQ2) set CellsIC2 = 0 and cells 17-32 to 0x0000 with fault bit 1 = 0 (not an error, just not present).

---

### Balance Command (Type 0xA)

**CAN ID:** `0x500 | slave_id` (slave_id = 1-8)

```
Byte:   0      1      2      3      4
     ┌──────┬──────┬──────┬──────┬──────┐
     │BalMsk│BalMsk│BalMsk│BalMsk│Buzzer│
     │ [7:0]│[15:8]│[23:16]│[31:24]│Code │
     └──────┴──────┴──────┴──────┴──────┘
```

**DLC:** 5 bytes

**Fields:**
| Bytes | Field | Description |
|-------|-------|-------------|
| 0-3 | BalanceMask | 32-bit bitmap, bit N = balance cell N |
| 4 | BuzzerCode | Buzzer beep code (see table below) |

Master is responsible for respecting BQ chip limits - only set bits for cells that should actually balance.

**Buzzer Beep Codes (Byte 4):**

| Code | Name | Beep Pattern |
|------|------|-------------|
| 0x00 | NONE | No beep (stop any active error pattern) |
| 0x01 | POWER_ON | 2 short beeps |
| 0x02 | POWER_OFF | 1 long beep |
| 0x03 | CHARGE_COMPLETE | 3 short beeps |
| 0x04 | SHUTDOWN | 4 fast beeps |
| 0x10 | ERR_OVER_TEMP | 1 beep, pause, repeat |
| 0x11 | ERR_CELL_HIGH | 2 beeps, pause, repeat |
| 0x12 | ERR_CELL_LOW | 3 beeps, pause, repeat |
| 0x13 | ERR_OVERCURRENT | 4 beeps, pause, repeat |
| 0x14 | ERR_BQ_COMM | 5 beeps, pause, repeat |

All beep codes (0x01-0x14) play once per received command. For error alerts, the master re-sends the beep code with each balance command to sustain the alert.

**Balance Watchdog:** Slave stops all balancing if no balance command received for **10 seconds**. Master must periodically resend balance command (even if unchanged) to maintain balancing.

**Backwards Compatibility:** Slave accepts both 4-byte (old firmware, no buzzer) and 5-byte (new firmware, with buzzer) balance commands. If byte 4 is missing, slave treats buzzer code as 0x00 (NONE).

---

## Timing

| Parameter | Value | Description |
|-----------|-------|-------------|
| Broadcast interval | 100ms | Slaves send all data every 100ms |
| Bus access | Automatic | CAN hardware handles arbitration - sends when bus is free |
| Slave timeout | 500ms | Master marks slave offline if no data |
| Balance update | As needed | Master sends when balance mask changes |
| Balance watchdog | 10s | Slave stops balancing if no command received |

### Transmission Sequence (per slave, every 100ms)

Each slave sends its messages back-to-back as fast as the CAN bus allows:

```
Cell voltages 1-4   (if cells >= 1)
Cell voltages 5-8   (if cells >= 5)
Cell voltages 9-12  (if cells >= 9)
Cell voltages 13-16 (if cells >= 13)
Cell voltages 17-20 (if cells >= 17)
Cell voltages 21-24 (if cells >= 21)
Cell voltages 25-28 (if cells >= 25)
Cell voltages 29-32 (if cells >= 29)
Temperatures
Status
```

**Message count depends on cell configuration:**
| Cells | Cell Messages | Total Messages | Messages/sec |
|-------|---------------|----------------|--------------|
| 12    | 3             | 5              | 50           |
| 16    | 4             | 6              | 60           |
| 20    | 5             | 7              | 70           |
| 32    | 8             | 10             | 100          |

**No artificial delays needed.** CAN arbitration automatically handles bus access:
- If bus is idle → transmit immediately
- If bus is busy → wait until idle, then transmit
- If collision → lower CAN ID wins, other retries immediately when bus is free

Messages from multiple slaves naturally interleave without explicit coordination.

---

## Balance Settle Synchronization

### Problem
BQ76952 balancing FETs cause voltage measurement errors while active. The internal balancing resistor draws current through the cell, causing an IR drop that makes the cell appear at a different voltage than its true open-circuit voltage. Using these "dirty" readings for balance decisions causes oscillation or over-balancing.

### Solution: Master-Driven Settle Cycle
The master coordinates a settle period before reading voltages for balance decisions. This mirrors the approach used by the VBMS32 (Harmony32) firmware.

**Balance cycle (repeated while balancing is active):**

```
1. Master sends zero balance mask to ALL slaves     (CAN TX, ~1ms)
2. Master stops local BQ76952 balancing             (I2C, ~1ms)
3. All devices settle for 2 seconds                 (FET-induced voltage error dissipates)
4. Slaves read settled voltages and broadcast them   (automatic, 10Hz continuous)
5. Master drains CAN buffer to get latest settled slave voltages
6. Master reads local BQ76952 voltages (also settled)
7. Master computes new balance masks from settled data
8. Master sends new balance commands to slaves       (CAN TX)
9. Balancing runs until next cycle (~0.2s cadence)
```

### Voltage-Settled Flag (Status Byte Bit 2)
Slaves track how long balance FETs have been off. After 20 consecutive loops (2 seconds at 10Hz) with zero balance bitmap, the slave sets bit 2 in the status message faults byte. This flag indicates to the master that the voltages being broadcast are accurate (not affected by balancing).

**Flag behavior:**
| Condition | Settled Flag |
|-----------|-------------|
| No balance command ever received (boot) | 1 (settled) |
| Balance bitmap is zero for >= 2s | 1 (settled) |
| Balance bitmap is non-zero | 0 (not settled) |
| Balance just stopped (< 2s ago) | 0 (not settled) |

The master can check this flag via `(master-get-slave-settled? slave-id)` to verify data quality before computing balance masks.

---

## CAN ID Quick Reference

### Slave→Master (Periodic Broadcasts, every 100ms)
```
Cell Voltages 1-4:   0x001-0x008 (slave 1-8)
Cell Voltages 5-8:   0x081-0x088
Cell Voltages 9-12:  0x101-0x108
Cell Voltages 13-16: 0x181-0x188
Cell Voltages 17-20: 0x201-0x208
Cell Voltages 21-24: 0x281-0x288
Cell Voltages 25-28: 0x301-0x308
Cell Voltages 29-32: 0x381-0x388
Temperatures:        0x401-0x408
Status:              0x481-0x488
```

### Master→Slave (Commands)
```
Balance (slave 1):   0x501
Balance (slave 2):   0x502
...
Balance (slave 8):   0x508
```

---

## Implementation Notes

### Master Integration
- Receives slave data via standard CAN RX
- Aggregates all cell voltages into single array (zeros = not present)
- Uses cells_ic1/cells_ic2 from Status message to know exact IC configuration
- Global min/max voltage across all slaves for balancing decisions
- Resends balance commands periodically (<10s) to maintain balancing
- Retries balance command if Status doesn't match expected

### Slave Implementation
- Initializes BQ76952 chips at startup, sets fault bits if init fails
- Reads BQ76952 at ~50ms internally
- Broadcasts only required cell messages every 100ms (optimizes CAN bus load)
- Sends cells_ic1 and cells_ic2 in Status message so master knows per-IC configuration
- Listens for balance commands matching its ID (0x501-0x508)
- Applies balance mask to BQ76952 CB_ACTIVE_CELLS register
- Stops balancing if no command received for 10 seconds (watchdog)

### Error Handling
| Condition | Slave Action | Master Action |
|-----------|--------------|---------------|
| BQ1 init failed | Set fault bit 0, send 0xFFFF for cells 1-16 | Detect via fault byte, mark slave faulty |
| BQ2 init failed | Set fault bit 1, send 0xFFFF for cells 17-32 | Detect via fault byte, mark slave faulty |
| No balance cmd (10s) | Stop all balancing | N/A (master should resend periodically) |
| Slave offline | N/A | Mark offline after 500ms timeout |
| Single-chip slave | Cells 17-32 = 0x0000, fault bit 1 = 0 | Normal operation (not an error) |

---
