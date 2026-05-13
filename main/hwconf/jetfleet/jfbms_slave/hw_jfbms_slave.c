/*
	Copyright 2025 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "hw_jfbms_slave.h"

#include "bq769x2_defs.h"

#include "heap.h"
#include "lbm_defines.h"
#include "main.h"
#include "i2c_compat.h"
#include "esp_sleep.h"
#include "lispif.h"
#include "lispbm.h"
#include "commands.h"
#include "utils.h"
#include "comm_can.h"
#include "bms.h"
#include "buffer.h"
#include "driver/ledc.h"

#include <math.h>
#include <stdint.h>

// Settings
#define BQ_ADDR_1 0x10  // BQ1 address after I2C address change during init
#define BQ_ADDR_2 0x08  // BQ2 stays at default address
#define I2C_SPEED 100000
#define I2C_MUTEX_TIMEOUT_MS 500

// Macros
#define M_CELLS (m_cells_ic1 + m_cells_ic2)

// Variables
static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t bq_mutex;
static unsigned int m_cells_ic1 = 16;
static unsigned int m_cells_ic2 = 0;
static uint16_t m_bal_state_ic1 = 0;
static uint16_t m_bal_state_ic2 = 0;

// Error messages
static char *error_comm_bq1 = "BQ1 communication error";
static char *error_comm_bq2 = "BQ2 communication error";

bool hw_can_get_filter_config(twai_mask_filter_config_t *cfg) {
	if (!cfg) {
		return false;
	}

	main_config_t *conf = (main_config_t *)&backup.config;
	uint32_t slave_id = (uint32_t)conf->slave_id & 0x7FU;
	uint32_t my_bal_id = 0x500U | slave_id;

	cfg->id = my_bal_id;
	cfg->mask = TWAI_STD_ID_MASK;
	cfg->is_ext = false;
	cfg->no_classic = false;
	cfg->no_fd = true;
	cfg->dual_filter = false;
	return true;
}

// ============================================================================
// BMS Master-Slave CAN Protocol (11-bit Standard IDs)
// ============================================================================
// CAN ID Format: (msg_type << 7) | slave_id
//
// Message Types (Slave -> Master):
//   0x0-0x7 = Cell voltages (4 cells per message, only sends messages needed)
//   0x8     = Temperatures (4 temps)
//   0x9     = Status (balance mask + faults)
//
// Message Types (Master -> Slave):
//   0xA     = Balance command (32-bit balance mask)
//
// Data Format:
//   - All multi-byte values are little-endian
//   - Cell voltages: uint16 mV (0x0000 = not populated, 0xFFFF = read error)
//   - Temperatures: int16 0.1°C (0x7FFF = not present/invalid)
// ============================================================================

// CAN ID macros per protocol spec
#define CAN_ID_CELLS(type, slave_id)  (((type) << 7) | (slave_id))
#define CAN_ID_TEMPS(slave_id)        (0x400 | (slave_id))
#define CAN_ID_STATUS(slave_id)       (0x480 | (slave_id))
#define CAN_ID_BAL_CMD(slave_id)      (0x500 | (slave_id))

// Forward declarations (functions defined later in file)
static bool subcommands_write16(uint8_t dev_addr, uint16_t command, uint16_t data);

// ============================================================================
// CAN Protocol TX Functions
// ============================================================================

/**
 * Send cell voltages (only messages needed for configured cell count)
 * Optimization: 12 cells sends 3 messages instead of 8, reducing CAN bus load
 * @param slave_id  Slave ID (1-8)
 * @param cells_mv  Array of cell voltages in mV (0 = not populated, 0xFFFF = error)
 */
static void can_send_all_cells(uint8_t slave_id, uint16_t *cells_mv) {
	// Only send messages needed for configured cells (4 cells per message)
	uint8_t num_msgs = (M_CELLS + 3) / 4;  // Round up: 12 cells = 3 msgs
	if (num_msgs > 8) num_msgs = 8;        // Cap at 8 messages (32 cells max)

	for (uint8_t msg_type = 0; msg_type < num_msgs; msg_type++) {
		uint8_t buf[8];
		uint8_t base_cell = msg_type * 4;

		// Pack 4 cells per message, little-endian
		for (uint8_t i = 0; i < 4; i++) {
			uint16_t v = cells_mv[base_cell + i];
			buf[i * 2]     = v & 0xFF;         // Low byte
			buf[i * 2 + 1] = (v >> 8) & 0xFF;  // High byte
		}

		comm_can_transmit_sid(CAN_ID_CELLS(msg_type, slave_id), buf, 8);
	}
}

/**
 * Send 4 temperatures (1 CAN message)
 * @param slave_id  Slave ID (1-8)
 * @param temps     Array of 4 temperatures in 0.1°C (0x7FFF = invalid)
 */
static void can_send_temps(uint8_t slave_id, int16_t *temps) {
	uint8_t buf[8];

	// Pack 4 temps, little-endian (T_BQ1_IC, T_BQ1_TS1, T_BQ2_IC, T_BQ2_TS1)
	for (uint8_t i = 0; i < 4; i++) {
		buf[i * 2]     = temps[i] & 0xFF;         // Low byte
		buf[i * 2 + 1] = (temps[i] >> 8) & 0xFF;  // High byte
	}

	comm_can_transmit_sid(CAN_ID_TEMPS(slave_id), buf, 8);
}

/**
 * Send status message (1 CAN message, 7 bytes)
 * @param slave_id   Slave ID (1-8)
 * @param bal_mask   32-bit balance bitmap
 * @param faults     Fault byte (bit0 = BQ1 init failed, bit1 = BQ2 init failed)
 * @param cells_ic1  Number of cells on BQ1 (0-16)
 * @param cells_ic2  Number of cells on BQ2 (0-16)
 */
static void can_send_status(uint8_t slave_id, uint32_t bal_mask, uint8_t faults,
		uint8_t cells_ic1, uint8_t cells_ic2) {
	uint8_t buf[7];

	// Balance mask, little-endian
	buf[0] = (bal_mask >> 0) & 0xFF;
	buf[1] = (bal_mask >> 8) & 0xFF;
	buf[2] = (bal_mask >> 16) & 0xFF;
	buf[3] = (bal_mask >> 24) & 0xFF;
	buf[4] = faults;
	buf[5] = cells_ic1;
	buf[6] = cells_ic2;

	comm_can_transmit_sid(CAN_ID_STATUS(slave_id), buf, 7);
}

// Get current balancing bitmap from both ICs
static uint32_t get_bal_bitmap(void) {
	return (uint32_t)m_bal_state_ic1 | ((uint32_t)m_bal_state_ic2 << 16);
}

// Apply balancing from bitmap
// Note: BQ76952 requires TOGGLE (0 then value) to reset internal ~18s timeout
// Just writing the same value does NOT reset the timer!
static bool apply_bal_bitmap(uint32_t bitmap) {
	uint16_t new_bal_ic1 = bitmap & 0xFFFF;
	uint16_t new_bal_ic2 = (bitmap >> 16) & 0xFFFF;
	bool res = true;

	// BQ1: Toggle - write 0 first, then actual value to reset internal timer
	subcommands_write16(BQ_ADDR_1, CB_ACTIVE_CELLS, 0);  // Clear first
	if (subcommands_write16(BQ_ADDR_1, CB_ACTIVE_CELLS, new_bal_ic1)) {
		m_bal_state_ic1 = new_bal_ic1;
	} else {
		res = false;
	}

	// BQ2: Toggle if present
	if (m_cells_ic2 > 0) {
		subcommands_write16(BQ_ADDR_2, CB_ACTIVE_CELLS, 0);  // Clear first
		if (subcommands_write16(BQ_ADDR_2, CB_ACTIVE_CELLS, new_bal_ic2)) {
			m_bal_state_ic2 = new_bal_ic2;
		} else {
			res = false;
		}
	}

	return res;
}

// Stop all balancing
static bool stop_all_balancing(void) {
	return apply_bal_bitmap(0);
}

static esp_err_t i2c_tx_rx(
	uint8_t addr, const uint8_t *write_buffer, size_t write_size,
	uint8_t *read_buffer, size_t read_size
) {

	if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		return ESP_ERR_TIMEOUT;
	}

	esp_err_t res;
	if (read_size > 0 && read_buffer != NULL) {
		if (write_size > 0 && write_buffer != NULL) {
			res = i2c_master_write_read_device(
				0, addr, write_buffer, write_size, read_buffer, read_size, 500
			);
		} else {
			res = i2c_master_read_from_device(
				0, addr, read_buffer, read_size, 500
			);
		}
	} else {
		res =
			i2c_master_write_to_device(0, addr, write_buffer, write_size, 500);
	}
	xSemaphoreGive(i2c_mutex);

	return res;
}

static uint8_t crc8(uint8_t *ptr, uint8_t len) {
	uint8_t i;
	uint8_t crc = 0;

	while (len-- != 0) {
		for (i = 0x80; i != 0; i /= 2) {
			if ((crc & 0x80) != 0) {
				crc *= 2;
				crc ^= 0x107;
			} else {
				crc *= 2;
			}

			if ((*ptr & i) != 0) {
				crc ^= 0x107;
			}
		}
		ptr++;
	}

	return (crc);
}

static bool bq_read_block(
	uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len
) {
	uint8_t read_data[2 * len];
	esp_err_t res          = i2c_tx_rx(dev_addr, &reg, 1, read_data, 2 * len);
	uint8_t *read_data_ptr = read_data;

	if (res != ESP_OK) {
		commands_printf_lisp("I2C Error: %d", res);
		return false;
	}

	uint8_t crcbuf[4];
	crcbuf[0]   = dev_addr << 1;
	crcbuf[1]   = reg;
	crcbuf[2]   = (dev_addr << 1) + 1;
	crcbuf[3]   = *read_data_ptr;
	uint8_t crc = crc8(crcbuf, 4);

	read_data_ptr++;
	if (crc != *read_data_ptr) {
		commands_printf_lisp("Bad CRC1");
		return false;
	} else {
		*buf = *(read_data_ptr - 1);
	}

	for (int i = 1; i < len; i++) {
		read_data_ptr++;
		crc = crc8(read_data_ptr, 1);
		read_data_ptr++;
		buf++;

		if (crc != *read_data_ptr) {
			commands_printf_lisp("Bad CRC2");
			return false;
		} else {
			*buf = *(read_data_ptr - 1);
		}
	}

	return true;
}

static bool bq_write_block(
	uint8_t dev_addr, uint8_t start_addr, uint8_t *buf, uint8_t len
) {
	uint8_t txbuf[2 * len + 2];
	txbuf[0] = dev_addr << 1;
	txbuf[1] = start_addr;
	txbuf[2] = buf[0];
	txbuf[3] = crc8(txbuf, 3);

	for (int i = 1; i < len; i++) {
		txbuf[2 + (2 * i)] = buf[i];
		txbuf[3 + (2 * i)] = crc8(&buf[i], 1);
	}

	esp_err_t res = i2c_tx_rx(dev_addr, txbuf + 1, 2 * len + 1, NULL, 0);

	return res == ESP_OK;
}

static uint8_t checksum(uint8_t *ptr, int len) {
	uint8_t sum = 0;

	for (int i = 0; i < len; i++) {
		sum += ptr[i];
	}

	return ~sum;
}

static bool bq_set_reg(
	uint8_t dev_addr, uint16_t reg_addr, uint32_t reg_data, uint8_t datalen
) {
	uint8_t TX_Buffer[2]  = {0x00, 0x00};
	uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	bool res = false;

	// TX_RegData in little endian format
	TX_RegData[0] = reg_addr & 0xff;
	TX_RegData[1] = (reg_addr >> 8) & 0xff;
	TX_RegData[2] = reg_data & 0xff; //1st byte of data

	switch (datalen) {
		case 1: //1 byte datalength
			bq_write_block(dev_addr, 0x3E, TX_RegData, 3);
			vTaskDelay(2);
			TX_Buffer[0] = checksum(TX_RegData, 3);
			TX_Buffer[1] = 0x05; //combined length of register address and data
			res          = bq_write_block(
                dev_addr, 0x60, TX_Buffer, 2
            ); // Write the checksum and length
			vTaskDelay(2);
			break;
		case 2: //2 byte datalength
			TX_RegData[3] = (reg_data >> 8) & 0xff;
			bq_write_block(dev_addr, 0x3E, TX_RegData, 4);
			vTaskDelay(2);
			TX_Buffer[0] = checksum(TX_RegData, 4);
			TX_Buffer[1] = 0x06; //combined length of register address and data
			res          = bq_write_block(
                dev_addr, 0x60, TX_Buffer, 2
            ); // Write the checksum and length
			vTaskDelay(2);
			break;
		case 4: //4 byte datalength, Only used for CCGain and Capacity Gain
			TX_RegData[3] = (reg_data >> 8) & 0xff;
			TX_RegData[4] = (reg_data >> 16) & 0xff;
			TX_RegData[5] = (reg_data >> 24) & 0xff;
			bq_write_block(dev_addr, 0x3E, TX_RegData, 6);
			vTaskDelay(2);
			TX_Buffer[0] = checksum(TX_RegData, 6);
			TX_Buffer[1] = 0x08; //combined length of register address and data
			res          = bq_write_block(
                dev_addr, 0x60, TX_Buffer, 2
            ); // Write the checksum and length
			vTaskDelay(2);
			break;
	}

	return res;
}

static bool bq_read_reg(
	uint8_t dev_addr, uint16_t reg_addr, uint32_t *reg_data, uint8_t datalen
) {
	uint8_t TX_RegData[2] = {0x00, 0x00};
	uint8_t RX_RegData[4] = {0x00, 0x00, 0x00, 0x00};

	if (datalen > 4) {
		datalen = 4;
	}

	bool res = false;

	// TX_RegData in little endian format
	TX_RegData[0] = reg_addr & 0xff;
	TX_RegData[1] = (reg_addr >> 8) & 0xff;

	bq_write_block(dev_addr, 0x3E, TX_RegData, 2);
	vTaskDelay(2);
	res = bq_read_block(dev_addr, 0x40, RX_RegData, datalen);

	if (res) {
		*reg_data = (((uint32_t)RX_RegData[3]) << 24)
			| (((uint32_t)RX_RegData[2]) << 16)
			| (((uint32_t)RX_RegData[1]) << 8)
			| (((uint32_t)RX_RegData[0]) << 0);
	} else {
		*reg_data = 0;
	}

	return res;
}

static int16_t command_read(uint8_t dev_addr, uint8_t command, bool *ok) {
	if (ok) {
		*ok = false;
	}
	uint8_t RX_data[2] = {0, 0};
	if (bq_read_block(dev_addr, command, RX_data, 2)) {
		if (ok) {
			*ok = true;
		}
		return (int16_t)(((uint16_t)RX_data[1] << 8) | (uint16_t)RX_data[0]);
	} else {
		return -1;
	}
}

static bool command_subcommands(uint8_t dev_addr, uint16_t command) {
	// For DEEPSLEEP/SHUTDOWN subcommand you will need to
	// call this function twice consecutively

	uint8_t TX_Reg[2] = {0x00, 0x00};

	// TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	bool res = bq_write_block(dev_addr, 0x3E, TX_Reg, 2);
	vTaskDelay(2);
	return res;
}

static bool __attribute__((unused)) subcommands_read16(
	uint8_t dev_addr, uint16_t command, uint16_t *result
) {
	uint8_t TX_Reg[2] = {0x00, 0x00};

	// TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	bool res = bq_write_block(dev_addr, 0x3E, TX_Reg, 2);

	if (!res) {
		return false;
	}

	vTaskDelay(2);

	uint8_t RX_data[2] = {0, 0};
	res                = bq_read_block(dev_addr, 0x40, RX_data, 2);

	if (!res) {
		return false;
	}

	*result = (int16_t)(((uint16_t)RX_data[1] << 8) | (uint16_t)RX_data[0]);

	return true;
}

static bool subcommands_write16(
	uint8_t dev_addr, uint16_t command, uint16_t data
) {
	uint8_t TX_Reg[4] = {0x00, 0x00, 0x00, 0x00};

	// TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;
	TX_Reg[2] = data & 0xff;
	TX_Reg[3] = (data >> 8) & 0xff;

	bool res = bq_write_block(dev_addr, 0x3E, TX_Reg, 4);

	if (!res) {
		return false;
	}

	vTaskDelay(1);

	TX_Reg[0] = checksum(TX_Reg, 4);
	TX_Reg[1] = 0x06;

	res = bq_write_block(dev_addr, 0x60, TX_Reg, 2);

	if (!res) {
		return false;
	}

	vTaskDelay(1);

	return true;
}

static bool subcommands_write8(
	uint8_t dev_addr, uint16_t command, uint8_t data
) {
	uint8_t TX_Reg[3] = {0x00, 0x00, 0x00};

	// TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;
	TX_Reg[2] = data;

	bool res = bq_write_block(dev_addr, 0x3E, TX_Reg, 3);

	if (!res) {
		return false;
	}

	vTaskDelay(1);

	TX_Reg[0] = checksum(TX_Reg, 3);
	TX_Reg[1] = 0x05;

	res = bq_write_block(dev_addr, 0x60, TX_Reg, 2);

	if (!res) {
		return false;
	}

	vTaskDelay(1);

	return true;
}

static uint32_t float_to_u(float number) {
	// Set subnormal numbers to 0 as they are not handled properly
	// using this method.
	if (fabsf(number) < 1.5e-38) {
		number = 0.0;
	}

	int e          = 0;
	float sig      = frexpf(number, &e);
	float sig_abs  = fabsf(sig);
	uint32_t sig_i = 0;

	if (sig_abs >= 0.5) {
		sig_i = (uint32_t)((sig_abs - 0.5f) * 2.0f * 8388608.0f);
		e    += 126;
	}

	uint32_t res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
	if (sig < 0) {
		res |= 1U << 31;
	}

	return res;
}

static void bq_init(uint8_t dev_addr) {
	command_subcommands(dev_addr, EXIT_DEEPSLEEP);
	command_subcommands(dev_addr, EXIT_DEEPSLEEP);
	vTaskDelay(10);

	//command_subcommands(dev_addr, BQ769x2_RESET);
	//vTaskDelay(60);

	// Disable all FETs (BQ76952 not used for FET control, only cell voltage monitoring)
	// 0x0F = all FETs OFF (bit 0 = DSG FET, bit 2 = CHG FET, bit 1,3 = other FETs)
	subcommands_write8(dev_addr, FET_CONTROL, 0x0F);

	command_subcommands(dev_addr, SET_CFGUPDATE);
	command_subcommands(dev_addr, SET_CFGUPDATE);

	// DPSLP_OT: 1
	// SHUT_TS2: 0
	// DPSLP_PD: 0
	// DPSLP_LDO: 1
	// DPSLP_LFO: 1
	// SLEEP: 0
	// OTSD: 1
	// FASTADC: 0
	// CB_LOOP_SLOW: 0
	// LOOP_SLOW: 0
	// WK_SPD: 0
	bq_set_reg(dev_addr, PowerConfig, 0b0010011010000000, 2);
	// Sometimes the first write has no effect. Do a few extra writes just in case...
	bq_set_reg(dev_addr, PowerConfig, 0b0010011010000000, 2);

	// REG0_EN: 1
	bq_set_reg(dev_addr, REG0Config, 0x01, 1);

	// REG1V: 6 (3.3v)
	// REG1_EN: 1
	bq_set_reg(dev_addr, REG12Config, 0b00001101, 1);

	// FETOptions
	// 5: FET_INIT_OFF
	// 4: PDSG_EN
	// 3: FET_CTRL_EN
	// 2: HOST_FET_EN
	// 1: SLEEPCHG
	// 0: SFET
	bq_set_reg(dev_addr, FETOptions, 0b00101100, 1);

	// Disabled
	bq_set_reg(dev_addr, CFETOFFPinConfig, 0x00, 1);
	bq_set_reg(dev_addr, DFETOFFPinConfig, 0x00, 1);

	// TS pin pull-up configuration:
	// 0b00111011 = 18k pull-up  | thermistor mode | ADC input (for NTC <= 50k)
	// 0b01111011 = 180k pull-up | thermistor mode | ADC input (for NTC >= 100k)
	main_config_t *cfg_bq = (main_config_t *)&backup.config;
	uint8_t ntcPinConfig;
	switch (cfg_bq->temp_res) {
		case NTC_RES_4_7K:  ntcPinConfig = 0b00111011; break;
		case NTC_RES_5K:    ntcPinConfig = 0b00111011; break;
		case NTC_RES_10K:   ntcPinConfig = 0b00111011; break;
		case NTC_RES_20K:   ntcPinConfig = 0b00111011; break;
		case NTC_RES_22K:   ntcPinConfig = 0b00111011; break;
		case NTC_RES_47K:   ntcPinConfig = 0b00111011; break;
		case NTC_RES_50K:   ntcPinConfig = 0b00111011; break;
		case NTC_RES_100K:  ntcPinConfig = 0b01111011; break;
		case NTC_RES_200K:  ntcPinConfig = 0b01111011; break;
		default:            ntcPinConfig = 0b00111011; break;
	}
	bq_set_reg(dev_addr, TS1Config, ntcPinConfig, 1);
	bq_set_reg(dev_addr, TS3Config, ntcPinConfig, 1);
	bq_set_reg(dev_addr, ALERTPinConfig, ntcPinConfig, 1);
	bq_set_reg(dev_addr, DCHGPinConfig, ntcPinConfig, 1);
	bq_set_reg(dev_addr, HDQPinConfig, 0b00111011, 1);

	// Disabled
	bq_set_reg(dev_addr, DDSGPinConfig, 0x00, 1);

	// Use all cells
	bq_set_reg(dev_addr, VCellMode, 0x0000, 2);

	// Disable automatic protections
	bq_set_reg(dev_addr, EnabledProtectionsA, 0x00, 1);
	bq_set_reg(dev_addr, EnabledProtectionsB, 0x00, 1);

	// Host-controlled balancing
	bq_set_reg(dev_addr, BalancingConfiguration, 0x00, 1);

	// Current gain
	float cc_gain = 7.4768 / (HW_R_SHUNT * 1000.0);
	bq_set_reg(dev_addr, CCGain, float_to_u(cc_gain), 4);
	bq_set_reg(dev_addr, CapacityGain, float_to_u(cc_gain * 298261.6178), 4);

	// Voltage and current reporting, 1 mV and 10 mA (range +- 320A)
	bq_set_reg(dev_addr, DAConfiguration, 0b00011110, 1);

	command_subcommands(dev_addr, EXIT_CFGUPDATE);

	vTaskDelay(10);

	command_subcommands(dev_addr, SLEEP_DISABLE);
}

// Extensions
static lbm_value ext_bms_init(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	m_bal_state_ic1 = 0;
	m_bal_state_ic2 = 0;

	unsigned int cells_ic1 = 16;
	if (argn >= 1) {
		cells_ic1 = lbm_dec_as_u32(args[0]);
	}

	unsigned int cells_ic2 = 0;  // Default to 0 (single chip mode)
	if (argn >= 2) {
		cells_ic2 = lbm_dec_as_u32(args[1]);
	}

	// Validation
	if (cells_ic1 < 3 || cells_ic1 > 16 || cells_ic2 > 16) {
		lbm_set_error_reason("Invalid cell combination");
		return ENC_SYM_TERROR;
	}

	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		lbm_set_error_reason("bq_mutex timeout in bms-init");
		return ENC_SYM_NIL;
	}

	// Disable BQ2 so only BQ1 is on the I2C bus during address change
	gpio_set_level(PIN_BQ1_EN, 0);  // Enable BQ1
	gpio_set_level(PIN_BQ2_EN, 1);  // Disable BQ2

	// Restart I2C while holding i2c_mutex. i2c_tx_rx() serializes transfers
	// through this mutex, but driver delete/install bypass it.
	if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		xSemaphoreGive(bq_mutex);
		lbm_set_error_reason("i2c_mutex timeout in bms-init");
		return ENC_SYM_NIL;
	}

	i2c_driver_delete(0);

	i2c_config_t conf = {
		.mode             = I2C_MODE_MASTER,
		.sda_io_num       = PIN_SDA,
		.scl_io_num       = PIN_SCL,
		.sda_pullup_en    = GPIO_PULLUP_ENABLE,
		.scl_pullup_en    = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_SPEED,
	};

	i2c_param_config(0, &conf);
	i2c_driver_install(0, conf.mode, 0, 0, 0);

	i2c_reset_tx_fifo(0);
	i2c_reset_rx_fifo(0);

	xSemaphoreGive(i2c_mutex);

	vTaskDelay(50);

	// Wake BQ1 if it was left in BQ-level DEEPSLEEP. In that mode its
	// configured I2C address is preserved, so try the post-init address first.
	command_subcommands(BQ_ADDR_1, EXIT_DEEPSLEEP);
	command_subcommands(BQ_ADDR_1, EXIT_DEEPSLEEP);
	vTaskDelay(pdMS_TO_TICKS(10));

	// With BQ2 disabled, an ACK at 0x10 can only be BQ1. If it is already
	// there, skip the reset/address-change path for faster warm/deepsleep boot.
	bool bq1_at_target = false;
	command_read(BQ_ADDR_1, Cell1Voltage, &bq1_at_target);

	if (!bq1_at_target) {
		// Cold-boot / default-address path: BQ1 is expected at 0x08.
		// If it was at 0x10, RESET moves it back to 0x08; if already at
		// 0x08, the reset at 0x10 simply NAKs. Allow the datasheet reset time.
		command_subcommands(BQ_ADDR_1, BQ769x2_RESET);
		vTaskDelay(pdMS_TO_TICKS(300));

		// Initialize BQ1 at its default address (0x08)
		bq_init(BQ_ADDR_2);

		// Change BQ1's I2C address from 0x08 to 0x10. I2CAddress takes
		// the 8-bit write address, so 0x10 << 1 = 0x20.
		command_subcommands(BQ_ADDR_2, SET_CFGUPDATE);
		if (!bq_set_reg(BQ_ADDR_2, I2CAddress, 0x20, 1)) {
			commands_printf_lisp("Could not update BQ1 I2C address");
		}
		command_subcommands(BQ_ADDR_2, EXIT_CFGUPDATE);
		// Apply the new I2C address immediately. Comm Type remains I2C.
		command_subcommands(BQ_ADDR_2, SWAP_COMM_MODE);
	}

	// Enable both BQs now that BQ1 has a unique address (0x10)
	gpio_set_level(PIN_BQ1_EN, 0);  // Enable BQ1 (at 0x10)
	gpio_set_level(PIN_BQ2_EN, 0);  // Enable BQ2 (at 0x08)
	vTaskDelay(50);

	// Initialize BQ2 at default address (0x08) if present
	if (cells_ic2 > 0) {
		bq_init(BQ_ADDR_2);
	}

	// Always refresh BQ1 configuration at its final address.
	bq_init(BQ_ADDR_1);

	m_cells_ic1 = cells_ic1;
	m_cells_ic2 = cells_ic2;

	// Test communication - BQ1 at 0x10, BQ2 at 0x08
	bool res = false;
	command_read(BQ_ADDR_1, Cell2Voltage, &res);
	if (m_cells_ic2 > 0) {
		bool res2 = false;
		command_read(BQ_ADDR_2, Cell2Voltage, &res2);
		res = res && res2;
	}

	xSemaphoreGive(bq_mutex);

	return res ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_hw_sleep(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
		lbm_set_error_reason("bq_mutex timeout in bms-sleep");
		return ENC_SYM_EERROR;
	}

	// Confirm both BQs respond before preparing them for DEEPSLEEP. If BQ1
	// lost its configured address after shutdown/power loss, fail early so the
	// caller can re-run bms-init instead of leaving the chips half-configured.
	{
		bool probe_ok = false;
		command_read(BQ_ADDR_1, Cell1Voltage, &probe_ok);
		if (!probe_ok) {
			xSemaphoreGive(bq_mutex);
			lbm_set_error_reason("BQ1 not responsive at 0x10 before sleep");
			return ENC_SYM_EERROR;
		}
		if (m_cells_ic2 > 0) {
			probe_ok = false;
			command_read(BQ_ADDR_2, Cell1Voltage, &probe_ok);
			if (!probe_ok) {
				xSemaphoreGive(bq_mutex);
				lbm_set_error_reason("BQ2 not responsive at 0x08 before sleep");
				return ENC_SYM_EERROR;
			}
		}
	}

	// Stop balancing on both chips
	m_bal_state_ic1 = 0;
	m_bal_state_ic2 = 0;

	if (!subcommands_write16(BQ_ADDR_1, CB_ACTIVE_CELLS, m_bal_state_ic1)) {
		goto exit_error1;
	}

	if (m_cells_ic2 > 0) {
		if (!subcommands_write16(BQ_ADDR_2, CB_ACTIVE_CELLS, m_bal_state_ic2)) {
			goto exit_error2;
		}
	}

	// Configure BQ1 for sleep (disable temp pull-ups, keep regulator on)
	if (!command_subcommands(BQ_ADDR_1, SET_CFGUPDATE)
		|| !bq_set_reg(BQ_ADDR_1, PowerConfig, 0b0010011010000000, 2)
		|| !bq_set_reg(BQ_ADDR_1, TS1Config, 0x00, 1)
		|| !bq_set_reg(BQ_ADDR_1, TS3Config, 0x00, 1)
		|| !command_subcommands(BQ_ADDR_1, EXIT_CFGUPDATE)) {
		goto exit_error1;
	}

	// Configure BQ2 for sleep if present
	if (m_cells_ic2 > 0) {
		if (!command_subcommands(BQ_ADDR_2, SET_CFGUPDATE)
			|| !bq_set_reg(BQ_ADDR_2, PowerConfig, 0b0010011010000000, 2)
			|| !bq_set_reg(BQ_ADDR_2, TS1Config, 0x00, 1)
			|| !bq_set_reg(BQ_ADDR_2, TS3Config, 0x00, 1)
			|| !command_subcommands(BQ_ADDR_2, EXIT_CFGUPDATE)) {
			goto exit_error2;
		}
	}

	// DEEPSLEEP must be sent twice in succession. Check both commands so a
	// NAK does not look like successful sleep entry to the Lisp caller.
	if (!command_subcommands(BQ_ADDR_1, DEEPSLEEP) ||
		!command_subcommands(BQ_ADDR_1, DEEPSLEEP)) {
		goto exit_error1;
	}

	if (m_cells_ic2 > 0) {
		if (!command_subcommands(BQ_ADDR_2, DEEPSLEEP) ||
			!command_subcommands(BQ_ADDR_2, DEEPSLEEP)) {
			goto exit_error2;
		}
	}

	xSemaphoreGive(bq_mutex);
	return ENC_SYM_TRUE;

exit_error1:
	xSemaphoreGive(bq_mutex);
	lbm_set_error_reason(error_comm_bq1);
	return ENC_SYM_EERROR;

exit_error2:
	xSemaphoreGive(bq_mutex);
	lbm_set_error_reason(error_comm_bq2);
	return ENC_SYM_EERROR;
}

static lbm_value ext_get_vcells(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	lbm_value vc_list = ENC_SYM_NIL;

	// Read BQ1 cells (at address 0x10)
	for (int i = 0; i < m_cells_ic1; i++) {
		bool ok = false;
		int res = command_read(BQ_ADDR_1, Cell1Voltage + i * 2, &ok);
		if (ok) {
			vc_list = lbm_cons(lbm_enc_float((float)res / 1000.0), vc_list);
		} else {
			lbm_set_error_reason(error_comm_bq1);
			return ENC_SYM_EERROR;
		}
	}

	// Read BQ2 cells if present (at address 0x08)
	if (m_cells_ic2 > 0) {
		for (int i = 0; i < m_cells_ic2; i++) {
			bool ok = false;
			int res = command_read(BQ_ADDR_2, Cell1Voltage + i * 2, &ok);
			if (ok) {
				vc_list = lbm_cons(lbm_enc_float((float)res / 1000.0), vc_list);
			} else {
				lbm_set_error_reason(error_comm_bq2);
				return ENC_SYM_EERROR;
			}
		}
	}

	return lbm_list_destructive_reverse(vc_list);
}

#define NTC_TEMP(res, ntc_res, beta)                                           \
	(1.0 / ((logf((res) / (ntc_res)) / (beta)) + (1.0 / 298.15)) - 273.15)
#define NTC_RES(volts) (18.0e3 / (1.8 / volts - 1.0) - 500.0)
// Return 999.0 for invalid NTC (will be converted to 0x7FFF in broadcast)
#define NTC_INVALID_MARKER 999.0f
#define NAN_TO_INVALID(x)  (UTILS_IS_NAN(x) ? NTC_INVALID_MARKER : x)

static lbm_value ext_get_temps(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	lbm_value ts_list = ENC_SYM_NIL;
	bool ok           = false;

	// Read BQ1 internal temperature (at address 0x10)
	ts_list = lbm_cons(
		lbm_enc_float(
			(float)command_read(BQ_ADDR_1, IntTemperature, &ok) * 0.1 - 273.15
		),
		ts_list
	);
	if (!ok) {
		goto exit_error1;
	}

	// Multiply by 256 as only 16 of the 24 bits are used
	const float counts_to_volts = 0.358e-6 * 256.0;

	main_config_t *cfg = (main_config_t *)&backup.config;
	float ntc_beta = (float)cfg->temp_beta;
	float ntc_res = 0.0;
	switch (cfg->temp_res) {
		case NTC_RES_4_7K:  ntc_res = 4700.0;   break;
		case NTC_RES_5K:    ntc_res = 5000.0;   break;
		case NTC_RES_10K:   ntc_res = 10000.0;  break;
		case NTC_RES_20K:   ntc_res = 20000.0;  break;
		case NTC_RES_22K:   ntc_res = 22000.0;  break;
		case NTC_RES_47K:   ntc_res = 47000.0;  break;
		case NTC_RES_50K:   ntc_res = 50000.0;  break;
		case NTC_RES_100K:  ntc_res = 100000.0; break;
		case NTC_RES_200K:  ntc_res = 200000.0; break;
		default:            ntc_res = 10000.0;  break;
	}

	// Read BQ1 TS1 (cell NTC on BQ1)
	float v1 = (float)command_read(BQ_ADDR_1, TS1Temperature, &ok) * counts_to_volts;
	if (!ok) {
		goto exit_error1;
	}
	ts_list = lbm_cons(
		lbm_enc_float(NAN_TO_INVALID(NTC_TEMP(NTC_RES(v1), ntc_res, ntc_beta))), ts_list
	);

	// Read BQ2 internal temperature if present (at address 0x08)
	if (m_cells_ic2 > 0) {
		ts_list = lbm_cons(
			lbm_enc_float(
				(float)command_read(BQ_ADDR_2, IntTemperature, &ok) * 0.1 - 273.15
			),
			ts_list
		);
		if (!ok) {
			goto exit_error2;
		}
	} else {
		// BQ2 not present - mark as invalid
		ts_list = lbm_cons(lbm_enc_float(NTC_INVALID_MARKER), ts_list);
	}

	// Read BQ2 TS1 (cell NTC on BQ2) if present
	if (m_cells_ic2 > 0) {
		float v2 = (float)command_read(BQ_ADDR_2, TS1Temperature, &ok) * counts_to_volts;
		if (!ok) {
			goto exit_error2;
		}
		ts_list = lbm_cons(
			lbm_enc_float(NAN_TO_INVALID(NTC_TEMP(NTC_RES(v2), ntc_res, ntc_beta))), ts_list
		);
	} else {
		ts_list = lbm_cons(lbm_enc_float(NTC_INVALID_MARKER), ts_list);
	}

	return lbm_list_destructive_reverse(ts_list);

exit_error1:
	lbm_set_error_reason(error_comm_bq1);
	return ENC_SYM_EERROR;

exit_error2:
	lbm_set_error_reason(error_comm_bq2);
	return ENC_SYM_EERROR;
}

static lbm_value ext_get_vout(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	bool ok = false;
	int16_t res = command_read(BQ_ADDR_1, LDPinVoltage, &ok);
	if (!ok) {
		lbm_set_error_reason(error_comm_bq1);
		return ENC_SYM_EERROR;
	}

	return lbm_enc_float((float)res / 100.0);
}

static lbm_value ext_get_vstack(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	bool ok = false;
	int16_t res = command_read(BQ_ADDR_1, StackVoltage, &ok);
	if (!ok) {
		lbm_set_error_reason(error_comm_bq1);
		return ENC_SYM_EERROR;
	}

	return lbm_enc_float((float)res / 100.0);
}

static lbm_value ext_set_bal(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	unsigned int ch = lbm_dec_as_u32(args[0]);
	int state       = lbm_dec_as_i32(args[1]);
	bool res        = false;

	if (ch < m_cells_ic1) {
		// Control BQ1 (at address 0x10)
		if (state) {
			m_bal_state_ic1 |= (1 << ch);
		} else {
			m_bal_state_ic1 &= ~(1 << ch);
		}

		res = subcommands_write16(BQ_ADDR_1, CB_ACTIVE_CELLS, m_bal_state_ic1);
		if (!res) {
			lbm_set_error_reason(error_comm_bq1);
		}
	} else if ((ch - m_cells_ic1) < m_cells_ic2) {
		// Control BQ2 (at address 0x08)
		unsigned int local_ch = ch - m_cells_ic1;
		if (state) {
			m_bal_state_ic2 |= (1 << local_ch);
		} else {
			m_bal_state_ic2 &= ~(1 << local_ch);
		}

		res = subcommands_write16(BQ_ADDR_2, CB_ACTIVE_CELLS, m_bal_state_ic2);
		if (!res) {
			lbm_set_error_reason(error_comm_bq2);
		}
	}

	return res ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

static lbm_value ext_get_bal(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	unsigned int ch = lbm_dec_as_u32(args[0]);
	int res         = -1;

	if (ch < m_cells_ic1) {
		res = (m_bal_state_ic1 >> ch) & 0x01;
	} else if ((ch - m_cells_ic1) < m_cells_ic2) {
		res = (m_bal_state_ic2 >> (ch - m_cells_ic1)) & 0x01;
	}

	return lbm_enc_i(res);
}

static lbm_value ext_direct_cmd(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	uint8_t addr = BQ_ADDR_1;
	if (lbm_dec_as_i32(args[0]) == 2) {
		addr = BQ_ADDR_2;
	}

	bool ok = false;
	int res = command_read(addr, lbm_dec_as_u32(args[1]), &ok);
	if (ok) {
		return lbm_enc_i(res);
	} else {
		lbm_set_error_reason(
			addr == BQ_ADDR_1 ? error_comm_bq1 : error_comm_bq2
		);
		return ENC_SYM_EERROR;
	}
}

static lbm_value ext_subcmd_cmdonly(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	uint8_t addr = BQ_ADDR_1;
	if (lbm_dec_as_i32(args[0]) == 2) {
		addr = BQ_ADDR_2;
	}

	return lbm_enc_i(command_subcommands(addr, lbm_dec_as_u32(args[1])));
}

static lbm_value ext_read_reg(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(3);

	uint8_t addr = BQ_ADDR_1;
	if (lbm_dec_as_i32(args[0]) == 2) {
		addr = BQ_ADDR_2;
	}

	int reg = lbm_dec_as_i32(args[1]);
	int len = lbm_dec_as_i32(args[2]);

	uint32_t reg_data = 0;
	bool ok           = bq_read_reg(addr, reg, &reg_data, len);

	if (ok) {
		return lbm_enc_u32(reg_data);
	} else {
		lbm_set_error_reason(
			addr == BQ_ADDR_1 ? error_comm_bq1 : error_comm_bq2
		);
		return ENC_SYM_EERROR;
	}
}

static lbm_value ext_write_reg(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(4);

	uint8_t addr = BQ_ADDR_1;
	if (lbm_dec_as_i32(args[0]) == 2) {
		addr = BQ_ADDR_2;
	}

	int reg       = lbm_dec_as_i32(args[1]);
	uint32_t data = lbm_dec_as_u32(args[2]);
	int len       = lbm_dec_as_i32(args[3]);

	bool ok = bq_set_reg(addr, reg, data, len);

	if (ok) {
		return ENC_SYM_TRUE;
	} else {
		lbm_set_error_reason(
			addr == BQ_ADDR_1 ? error_comm_bq1 : error_comm_bq2
		);
		return ENC_SYM_EERROR;
	}
}

typedef struct {
	lbm_uint slave_id;
	lbm_uint cells_ic1;
	lbm_uint cells_ic2;
} config_syms;

static config_syms syms_cfg = {0};

static bool compare_symbol(lbm_uint sym, lbm_uint *comp) {
	if (*comp == 0) {
		if (comp == &syms_cfg.slave_id) {
			lbm_add_symbol_const("slave_id", comp);
		} else if (comp == &syms_cfg.cells_ic1) {
			lbm_add_symbol_const("cells_ic1", comp);
		} else if (comp == &syms_cfg.cells_ic2) {
			lbm_add_symbol_const("cells_ic2", comp);
		}
	}

	return *comp == sym;
}

static lbm_value get_or_set_i(bool set, int *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_i32(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_i(*val);
	}
}

static lbm_value bms_get_set_param(bool set, lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_EERROR;

	lbm_value set_arg = 0;
	if (set && argn >= 1) {
		set_arg = args[argn - 1];
		argn--;

		if (!lbm_is_number(set_arg)) {
			lbm_set_error_reason((char *)lbm_error_str_no_number);
			return ENC_SYM_EERROR;
		}
	}

	if (argn != 1 && argn != 2) {
		return res;
	}

	if (lbm_type_of(args[0]) != LBM_TYPE_SYMBOL) {
		return res;
	}

	lbm_uint name      = lbm_dec_sym(args[0]);
	main_config_t *cfg = (main_config_t *)&backup.config;

	if (compare_symbol(name, &syms_cfg.slave_id)) {
		res = get_or_set_i(set, &cfg->slave_id, &set_arg);
	} else if (compare_symbol(name, &syms_cfg.cells_ic1)) {
		res = get_or_set_i(set, &cfg->cells_ic1, &set_arg);
	} else if (compare_symbol(name, &syms_cfg.cells_ic2)) {
		res = get_or_set_i(set, &cfg->cells_ic2, &set_arg);
	}

	return res;
}

static lbm_value ext_bms_get_param(lbm_value *args, lbm_uint argn) {
	return bms_get_set_param(false, args, argn);
}

static lbm_value ext_bms_set_param(lbm_value *args, lbm_uint argn) {
	return bms_get_set_param(true, args, argn);
}

static lbm_value ext_bms_store_cfg(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	main_store_backup_data();
	return ENC_SYM_TRUE;
}

// I2C Overrides

static lbm_value ext_i2c_start(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return ENC_SYM_TRUE;
}

static lbm_value ext_i2c_tx_rx(lbm_value *args, lbm_uint argn) {
	if (argn != 2 && argn != 3) {
		return ENC_SYM_EERROR;
	}

	uint16_t addr  = 0;
	size_t txlen   = 0;
	size_t rxlen   = 0;
	uint8_t *txbuf = 0;
	uint8_t *rxbuf = 0;

	const unsigned int max_len = 20;
	uint8_t to_send[max_len];

	if (!lbm_is_number(args[0])) {
		return ENC_SYM_EERROR;
	}
	addr = lbm_dec_as_u32(args[0]);

	if (lbm_is_array_r(args[1])) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[1]);
		txbuf                     = (uint8_t *)array->data;
		txlen                     = array->size;
	} else {
		lbm_value curr = args[1];
		while (lbm_is_cons(curr)) {
			lbm_value arg = lbm_car(curr);

			if (lbm_is_number(arg)) {
				to_send[txlen++] = lbm_dec_as_u32(arg);
			} else {
				return ENC_SYM_EERROR;
			}

			if (txlen == max_len) {
				break;
			}

			curr = lbm_cdr(curr);
		}

		if (txlen > 0) {
			txbuf = to_send;
		}
	}

	if (argn >= 3 && lbm_is_array_rw(args[2])) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[2]);
		rxbuf                     = (uint8_t *)array->data;
		rxlen                     = array->size;
	}

	return lbm_enc_i(i2c_tx_rx(addr, txbuf, txlen, rxbuf, rxlen));
}

static lbm_value ext_i2c_detect_addr(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint8_t address = lbm_dec_as_u32(args[0]);
	if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		return ENC_SYM_NIL;
	}
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(0, cmd, 50 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	xSemaphoreGive(i2c_mutex);

	return ret == ESP_OK ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_bms_fw_version(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_i(6);
}

static lbm_value ext_set_buzzer(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	gpio_set_level(PIN_BUZZER, lbm_dec_as_i32(args[0]));
	return ENC_SYM_TRUE;
}

// ============================================================================
// CAN Protocol LispBM Extensions
// ============================================================================

// Track fault state for status messages
static volatile uint8_t m_fault_flags = 0;
// Voltage-settled flag: 1 = balance FETs off long enough for accurate readings
static volatile uint8_t m_settled_flag = 1;  // Start settled (no balancing at boot)

// (bms-set-fault-flags flags)
// Set fault flags (bit0 = BQ1 init failed, bit1 = BQ2 init failed)
static lbm_value ext_set_fault_flags(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	m_fault_flags = lbm_dec_as_u32(args[0]) & 0xFF;
	return ENC_SYM_TRUE;
}

// (bms-set-settled-flag flag)
// Set voltage-settled flag (1 = settled, 0 = not settled)
// Included as bit 2 in the faults byte of status CAN message
static lbm_value ext_set_settled_flag(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	m_settled_flag = lbm_dec_as_i32(args[0]) != 0 ? 1 : 0;
	return ENC_SYM_TRUE;
}

// (bms-broadcast-all slave-id cells-list temps-list bq1-ok bq2-ok)
// Broadcast all data to master per protocol (up to 8 cell messages + 1 temp + 1 status)
static lbm_value ext_broadcast_all(lbm_value *args, lbm_uint argn) {
	if (argn < 3) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_number(args[0]) || !lbm_is_list(args[1]) || !lbm_is_list(args[2])) {
		return ENC_SYM_EERROR;
	}

	uint8_t slave_id = lbm_dec_as_u32(args[0]);

	// Extract cell voltages into 32-element array
	// 0 = not populated, 0xFFFF = read error
	uint16_t cells_mv[32] = {0};
	uint8_t num_cells = 0;
	lbm_value curr = args[1];

	while (lbm_is_cons(curr) && num_cells < 32) {
		lbm_value cell = lbm_car(curr);
		if (lbm_is_number(cell)) {
			float v = lbm_dec_as_float(cell);
			if (v < 0) {
				cells_mv[num_cells] = 0xFFFF;  // Error marker
			} else {
				cells_mv[num_cells] = (uint16_t)(v * 1000.0f);  // Convert V to mV
			}
			num_cells++;
		}
		curr = lbm_cdr(curr);
	}

	// Extract temperatures into 4-element array
	// 0x7FFF = not present/invalid
	int16_t temps[4] = {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF};
	uint8_t num_temps = 0;
	curr = args[2];

	while (lbm_is_cons(curr) && num_temps < 4) {
		lbm_value temp = lbm_car(curr);
		if (lbm_is_number(temp)) {
			float t = lbm_dec_as_float(temp);
			if (t < -40.0f || t > 120.0f) {
				temps[num_temps] = 0x7FFF;  // Invalid marker
			} else {
				temps[num_temps] = (int16_t)(t * 10.0f);  // Convert to 0.1°C
			}
			num_temps++;
		}
		curr = lbm_cdr(curr);
	}

	// Get fault flags from optional args or use stored value
	uint8_t faults = m_fault_flags;
	if (argn >= 5) {
		bool bq1_ok = lbm_dec_as_i32(args[3]) != 0;
		bool bq2_ok = lbm_dec_as_i32(args[4]) != 0;
		faults = 0;
		if (!bq1_ok) faults |= 0x01;
		if (!bq2_ok) faults |= 0x02;
	}

	// Include voltage-settled flag (bit 2)
	if (m_settled_flag) faults |= 0x04;

	// Send all messages per protocol
	// TX queue is 20 messages, we send 10, so no delays needed
	can_send_all_cells(slave_id, cells_mv);
	can_send_temps(slave_id, temps);
	can_send_status(slave_id, get_bal_bitmap(), faults,
			(uint8_t)m_cells_ic1, (uint8_t)m_cells_ic2);

	return ENC_SYM_TRUE;
}

// (bms-set-bal-bitmap ic1-mask ic2-mask)
// Set balancing state from two 16-bit masks (avoids LispBM 28-bit integer overflow)
// C code combines: bitmap = ic1_mask | (ic2_mask << 16)
static lbm_value ext_set_bal_bitmap(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	uint32_t ic1_mask = lbm_dec_as_u32(args[0]) & 0xFFFF;
	uint32_t ic2_mask = lbm_dec_as_u32(args[1]) & 0xFFFF;
	uint32_t bitmap = ic1_mask | (ic2_mask << 16);

	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		lbm_set_error_reason("bq_mutex timeout in bms-set-bal-bitmap");
		return ENC_SYM_EERROR;
	}
	bool res = apply_bal_bitmap(bitmap);
	xSemaphoreGive(bq_mutex);

	return res ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

// (bms-get-bal-bitmap)
// Get current balancing state as 32-bit bitmap
static lbm_value ext_get_bal_bitmap(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_u32(get_bal_bitmap());
}

// (bms-stop-balancing)
// Stop all cell balancing
static lbm_value ext_stop_balancing(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		lbm_set_error_reason("bq_mutex timeout in bms-stop-balancing");
		return ENC_SYM_EERROR;
	}
	bool res = stop_all_balancing();
	xSemaphoreGive(bq_mutex);

	return res ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

// (bms-set-bal-bitmap-demo ic1-mask ic2-mask)
// Set balance mask directly without writing to BQ chips (for demo/testing)
// Takes two 16-bit masks to avoid LispBM 28-bit integer overflow
static lbm_value ext_set_bal_bitmap_demo(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	m_bal_state_ic1 = lbm_dec_as_u32(args[0]) & 0xFFFF;
	m_bal_state_ic2 = lbm_dec_as_u32(args[1]) & 0xFFFF;

	return ENC_SYM_TRUE;
}

// (bms-get-slave-id)
// Get slave ID from configuration (set via VESC Tool -> JFBMS Slave -> Slave ID)
static lbm_value ext_get_slave_id(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	main_config_t *cfg = (main_config_t *)&backup.config;
	return lbm_enc_i(cfg->slave_id);
}

// ============================================================================
// Buzzer Beep Task
// ============================================================================

#define BUZZER_LEDC_CHANNEL  LEDC_CHANNEL_0
#define BUZZER_LEDC_TIMER    LEDC_TIMER_0
#define BUZZER_FREQ_HZ       4000
#define BUZZER_DUTY_BITS     LEDC_TIMER_10_BIT
#define BUZZER_DUTY_ON       512  // 50% of 1024

static volatile uint8_t buzzer_beep_count = 0;
static volatile uint16_t buzzer_beep_duration_ms = 100;
static TaskHandle_t buzzer_task_handle = NULL;

static void buzzer_pwm_on(void) {
	ledc_timer_config_t timer_cfg = {
		.speed_mode       = LEDC_LOW_SPEED_MODE,
		.timer_num        = BUZZER_LEDC_TIMER,
		.duty_resolution  = BUZZER_DUTY_BITS,
		.freq_hz          = BUZZER_FREQ_HZ,
		.clk_cfg          = LEDC_AUTO_CLK,
	};
	ledc_timer_config(&timer_cfg);

	ledc_channel_config_t ch_cfg = {
		.speed_mode     = LEDC_LOW_SPEED_MODE,
		.channel        = BUZZER_LEDC_CHANNEL,
		.timer_sel      = BUZZER_LEDC_TIMER,
		.intr_type      = LEDC_INTR_DISABLE,
		.gpio_num       = PIN_BUZZER,
		.duty           = BUZZER_DUTY_ON,
		.hpoint         = 0,
	};
	ledc_channel_config(&ch_cfg);
}

static void buzzer_pwm_off(void) {
	ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, 0);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);
	ledc_stop(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, 0);
	gpio_set_level(PIN_BUZZER, 0);
}

static void buzzer_task(void *arg) {
	(void)arg;
	while (true) {
		// Wait for notification
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		uint8_t count = buzzer_beep_count;
		uint16_t dur = buzzer_beep_duration_ms;

		for (uint8_t i = 0; i < count; i++) {
			buzzer_pwm_on();
			vTaskDelay(pdMS_TO_TICKS(dur));
			buzzer_pwm_off();
			if (i < count - 1) {
				vTaskDelay(pdMS_TO_TICKS(dur));
			}
		}
	}
}

static void buzzer_request_beeps(uint8_t count, uint16_t duration_ms) {
	buzzer_beep_count = count;
	buzzer_beep_duration_ms = duration_ms;
	if (buzzer_task_handle) {
		xTaskNotifyGive(buzzer_task_handle);
	}
}

// (buzzer-beep count duration-ms)
// Low-level: play count beeps of given duration via FreeRTOS buzzer task
static lbm_value ext_buzzer_beep(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);
	uint8_t count = lbm_dec_as_u32(args[0]);
	uint16_t duration_ms = lbm_dec_as_u32(args[1]);
	if (count == 0 || count > 20) return ENC_SYM_NIL;
	if (duration_ms == 0 || duration_ms > 5000) return ENC_SYM_NIL;
	buzzer_request_beeps(count, duration_ms);
	return ENC_SYM_TRUE;
}

// ============================================================================
// Direct CAN RX buffer for slave LispBM extensions
// ============================================================================

#define CAN_BUF_SIZE 64

typedef struct {
	uint32_t id;
	uint8_t data[8];
	uint8_t len;
} can_msg_t;

static can_msg_t can_rx_buffer[CAN_BUF_SIZE];
static volatile int can_rx_write = 0;
static volatile int can_rx_read = 0;
static volatile uint32_t can_rx_overflow = 0;

// Hardware CAN hook - called from comm_can.c for every received message.
// ID filtering is handled in TWAI hardware filter for slave builds.
void hw_can_rx_hook(uint32_t id, uint8_t *data, int len, bool is_ext) {
	if (is_ext) return;  // Only handle standard 11-bit IDs

	int next_write = (can_rx_write + 1) % CAN_BUF_SIZE;
	if (next_write == can_rx_read) {
		can_rx_overflow++;
		return;
	}

	can_rx_buffer[can_rx_write].id = id;
	can_rx_buffer[can_rx_write].len = len > 8 ? 8 : len;
	memcpy(can_rx_buffer[can_rx_write].data, data, can_rx_buffer[can_rx_write].len);
	can_rx_write = next_write;
}

// (slave-can-available) - Returns number of messages in buffer
static lbm_value ext_slave_can_available(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	int count = can_rx_write - can_rx_read;
	if (count < 0) count += CAN_BUF_SIZE;
	return lbm_enc_i(count);
}

// (slave-can-overflow) - Returns number of dropped messages due to full buffer
static lbm_value ext_slave_can_overflow(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_u32(can_rx_overflow);
}

// (slave-can-read) - Read one message from buffer, returns (id . data) or nil
static lbm_value ext_slave_can_read(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	if (can_rx_read == can_rx_write) {
		return ENC_SYM_NIL;  // Buffer empty
	}

	can_msg_t *msg = &can_rx_buffer[can_rx_read];
	can_rx_read = (can_rx_read + 1) % CAN_BUF_SIZE;

	// Create array for data
	lbm_value data_arr;
	if (!lbm_heap_allocate_array(&data_arr, msg->len)) {
		return ENC_SYM_NIL;
	}
	lbm_array_header_t *arr = (lbm_array_header_t *)lbm_car(data_arr);
	memcpy(arr->data, msg->data, msg->len);

	// Return (id . data)
	return lbm_cons(lbm_enc_u32(msg->id), data_arr);
}

// ============================================================================
// VESC BMS Data Integration (for VESC Tool display when connected to slave)
// ============================================================================

// (slave-update-vesc-bms cells-list temps-list)
// Update VESC BMS values for display in VESC Tool
// cells-list: list of cell voltages in V from (bms-get-vcells)
// temps-list: list of temperatures in C from (bms-get-temps)
static lbm_value ext_slave_update_vesc_bms(lbm_value *args, lbm_uint argn) {
	if (argn < 2) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_list(args[0]) || !lbm_is_list(args[1])) {
		return ENC_SYM_EERROR;
	}

	volatile bms_values *bms = bms_get_values();

	int num_cells = 0;
	float v_tot = 0.0f;
	float v_min = 9999.0f;
	float v_max = 0.0f;

	// Extract cell voltages from list (pass all values through, no filtering)
	uint32_t bal_bitmap = get_bal_bitmap();
	int ic1_cnt = m_cells_ic1;
	lbm_value curr = args[0];
	while (lbm_is_cons(curr) && num_cells < BMS_MAX_CELLS) {
		lbm_value cell = lbm_car(curr);
		if (lbm_is_number(cell)) {
			float v = lbm_dec_as_float(cell);
			bms->v_cell[num_cells] = v;
			// Balance mask is ic1[0:15] | ic2[16:31], map cell index to correct bit
			int bit = (num_cells < ic1_cnt) ? num_cells : (16 + num_cells - ic1_cnt);
			bms->bal_state[num_cells] = (bal_bitmap >> bit) & 1;
			v_tot += v;
			if (v < v_min) v_min = v;
			if (v > v_max) v_max = v;
			num_cells++;
		}
		curr = lbm_cdr(curr);
	}

	// Extract temperatures from list
	// Temp order: [0]=BQ1 IC, [1]=BQ1 TS1 (cell), [2]=BQ2 IC, [3]=BQ2 TS1 (cell)
	// Track cell temps (indices 1,3) and IC temps (indices 0,2) separately
	int num_temps = 0;
	int temp_idx = 0;
	float t_max_cell = -273.0f;
	float t_max_ic = -273.0f;
	curr = args[1];
	while (lbm_is_cons(curr) && num_temps < BMS_MAX_TEMPS) {
		lbm_value temp = lbm_car(curr);
		if (lbm_is_number(temp)) {
			float t = lbm_dec_as_float(temp);
			// Filter: -40..120°C valid range; 999.0 = NTC_INVALID_MARKER (no sensor / NaN)
			if (t >= -40.0f && t < 900.0f) {
				bms->temps_adc[num_temps] = t;
				// Indices 0,2 = IC die temps; indices 1,3 = cell NTC temps
				if (temp_idx == 0 || temp_idx == 2) {
					if (t > t_max_ic) t_max_ic = t;
				} else {
					if (t > t_max_cell) t_max_cell = t;
				}
				num_temps++;
			}
		}
		temp_idx++;
		curr = lbm_cdr(curr);
	}

	// Clear remaining cell slots
	for (int i = num_cells; i < BMS_MAX_CELLS; i++) {
		bms->v_cell[i] = 0.0f;
		bms->bal_state[i] = 0;
	}

	// Update cell totals
	bms->cell_num = num_cells;
	bms->v_tot = v_tot;
	bms->v_cell_min = (num_cells > 0) ? v_min : 0.0f;
	bms->v_cell_max = (num_cells > 0) ? v_max : 0.0f;

	// Update temperature data
	bms->temp_adc_num = num_temps;
	bms->temp_max_cell = (t_max_cell > -273.0f) ? t_max_cell : 0.0f;
	bms->temp_ic = (t_max_ic > -273.0f) ? t_max_ic : 0.0f;

	// Balancing state
	bms->is_balancing = (get_bal_bitmap() != 0) ? 1 : 0;

	// Slave ID from config
	main_config_t *cfg = (main_config_t *)&backup.config;
	bms->can_id = cfg->slave_id;

	// Update timestamp
	bms->update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

	// SOC estimate based on average cell voltage (simple linear approximation)
	float avg_v = (num_cells > 0) ? (v_tot / num_cells) : 3.7f;
	bms->soc = (avg_v - 3.0f) / (4.2f - 3.0f);
	if (bms->soc < 0.0f) bms->soc = 0.0f;
	if (bms->soc > 1.0f) bms->soc = 1.0f;

	bms->soh = 1.0f;

	return ENC_SYM_TRUE;
}


static void load_extensions(bool main_found) {
	if (main_found) {
		return;
	}

	memset(&syms_cfg, 0, sizeof(syms_cfg));

	// Wake up and initialize hardware
	lbm_add_extension("bms-init", ext_bms_init);

	// Put BMS hardware in sleep mode
	lbm_add_extension("bms-sleep", ext_hw_sleep);

	// Get list of cell voltages
	lbm_add_extension("bms-get-vcells", ext_get_vcells);

	// Get list of temperature readings
	lbm_add_extension("bms-get-temps", ext_get_temps);

	// Get output voltage after power switch
	lbm_add_extension("bms-get-vout", ext_get_vout);

	// Get stack voltage
	lbm_add_extension("bms-get-vstack", ext_get_vstack);

	// Set and get balancing state for cell
	lbm_add_extension("bms-set-bal", ext_set_bal);
	lbm_add_extension("bms-get-bal", ext_get_bal);

	// Buzzer control
	lbm_add_extension("bms-set-buzzer", ext_set_buzzer);
	lbm_add_extension("buzzer-beep", ext_buzzer_beep);

	// CAN protocol for master-slave communication (11-bit IDs)
	lbm_add_extension("bms-broadcast-all", ext_broadcast_all);
	lbm_add_extension("bms-set-fault-flags", ext_set_fault_flags);
	lbm_add_extension("bms-set-settled-flag", ext_set_settled_flag);
	lbm_add_extension("bms-set-bal-bitmap", ext_set_bal_bitmap);
	lbm_add_extension("bms-get-bal-bitmap", ext_get_bal_bitmap);
	lbm_add_extension("bms-stop-balancing", ext_stop_balancing);
	lbm_add_extension("bms-set-bal-bitmap-demo", ext_set_bal_bitmap_demo);
	lbm_add_extension("bms-get-slave-id", ext_get_slave_id);

	// Direct CAN buffer extensions for the slave protocol
	lbm_add_extension("slave-can-available", ext_slave_can_available);
	lbm_add_extension("slave-can-overflow", ext_slave_can_overflow);
	lbm_add_extension("slave-can-read", ext_slave_can_read);

	// VESC BMS integration (for VESC Tool display when connected to slave)
	lbm_add_extension("slave-update-vesc-bms", ext_slave_update_vesc_bms);

	// HW-specific commands
	lbm_add_extension("bms-direct-cmd", ext_direct_cmd);
	lbm_add_extension("bms-subcmd-cmdonly", ext_subcmd_cmdonly);
	lbm_add_extension("bms-read-reg", ext_read_reg);
	lbm_add_extension("bms-write-reg", ext_write_reg);

	// Configuration
	lbm_add_extension("bms-get-param", ext_bms_get_param);
	lbm_add_extension("bms-set-param", ext_bms_set_param);
	lbm_add_extension("bms-store-cfg", ext_bms_store_cfg);

	// Replace existing I2C-extensions
	lbm_add_extension("i2c-start", ext_i2c_start);
	lbm_add_extension("i2c-tx-rx", ext_i2c_tx_rx);
	lbm_add_extension("i2c-detect-addr", ext_i2c_detect_addr);

	lbm_add_extension("bms-fw-version", ext_bms_fw_version);
}

void hw_init(void) {
	i2c_mutex = xSemaphoreCreateMutex();
	bq_mutex  = xSemaphoreCreateMutex();

	// Disable VESC CAN protocol decoder - we only use our 11-bit protocol
	comm_can_use_vesc_decoder(false);

	gpio_config_t gpconf = {0};

	// Configure BQ communication enable pins (active LOW)
	// Start with only BQ1 enabled - BQ2 disabled until bms-init changes BQ1's I2C address
	gpio_set_level(PIN_BQ1_EN, 0);  // LOW = enabled
	gpio_set_level(PIN_BQ2_EN, 1);  // HIGH = disabled until address change

	gpconf.pin_bit_mask = BIT(PIN_BQ1_EN) | BIT(PIN_BQ2_EN);
	gpconf.intr_type    = GPIO_FLOATING;
	gpconf.mode         = GPIO_MODE_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en   = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);

	// Set levels again after config to ensure they're applied
	gpio_set_level(PIN_BQ1_EN, 0);  // LOW = enabled
	gpio_set_level(PIN_BQ2_EN, 1);  // HIGH = disabled until address change

	// Configure buzzer pin
	gpio_set_level(PIN_BUZZER, 0);
	gpconf.pin_bit_mask = BIT(PIN_BUZZER);
	gpconf.mode         = GPIO_MODE_OUTPUT;
	gpio_config(&gpconf);

	// Create buzzer beep task
	xTaskCreate(buzzer_task, "buzzer", 1024, NULL, 5, &buzzer_task_handle);

	// Initialize I2C
	i2c_config_t conf = {
		.mode             = I2C_MODE_MASTER,
		.sda_io_num       = PIN_SDA,
		.scl_io_num       = PIN_SCL,
		.sda_pullup_en    = GPIO_PULLUP_ENABLE,
		.scl_pullup_en    = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 100000,
	};

	i2c_param_config(0, &conf);
	i2c_driver_install(0, conf.mode, 0, 0, 0);

	lispif_add_ext_load_callback(load_extensions);
}
