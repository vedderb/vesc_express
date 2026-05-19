/*
	Copyright 2024 Benjamin Vedder	benjamin@vedder.se

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

#include HW_HEADER
#include "bq769x2_defs.h"

#include "main.h"
#include "i2c_compat.h"
#include "esp_sleep.h"
#include "lispif.h"
#include "lispbm.h"
#include "commands.h"
#include "utils.h"

#include <math.h>
#include <sys/time.h>

// Settings
#define BQ_ADDR_1 0x10
#define BQ_ADDR_2 0x08
#define I2C_SPEED 100000
#define BMS_INIT_ADDR_RETRIES 3

// Bound all I2C / BQ mutex waits so a stuck holder cannot deadlock every
// caller forever. If this ever times out we bail out with an error instead of
// blocking - combined with the task WDT this is what lets a stuck bus recover.
#define I2C_MUTEX_TIMEOUT_MS 500
#define BMS_BALANCE_OFF_RETRIES 3
#define BMS_BALANCE_OFF_RETRY_DELAY_MS 20

// Macros
#define M_CELLS (m_cells_ic1 + m_cells_ic2)

// Variables
static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t bq_mutex;
static unsigned int m_cells_ic1 = 16;
static unsigned int m_cells_ic2 = 16;
static uint16_t m_bal_state_ic1 = 0;
static uint16_t m_bal_state_ic2 = 0;

// Error messages
static char *error_comm_bq1 = "BQ1 communication error";
static char *error_comm_bq2 = "BQ2 communication error";

static void bms_set_chg_hw(bool enable) {
	gpio_set_level(PIN_PSW_EN, 1);
	gpio_set_level(PIN_CHG_EN, enable ? 1 : 0);
}

static void bms_clear_balance_state(void) {
	m_bal_state_ic1 = 0;
	m_bal_state_ic2 = 0;
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

static void i2c_bus_clear(void) {
	gpio_set_direction(PIN_SDA, GPIO_MODE_INPUT_OUTPUT_OD);
	gpio_set_direction(PIN_SCL, GPIO_MODE_INPUT_OUTPUT_OD);
	gpio_set_pull_mode(PIN_SDA, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(PIN_SCL, GPIO_PULLUP_ONLY);

	gpio_set_level(PIN_SDA, 1);
	gpio_set_level(PIN_SCL, 1);
	vTaskDelay(pdMS_TO_TICKS(1));

	for (int i = 0; i < 9 && gpio_get_level(PIN_SDA) == 0; i++) {
		gpio_set_level(PIN_SCL, 0);
		vTaskDelay(pdMS_TO_TICKS(1));
		gpio_set_level(PIN_SCL, 1);
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	// Generate a STOP condition in case a slave was left mid-transaction.
	gpio_set_level(PIN_SDA, 0);
	vTaskDelay(pdMS_TO_TICKS(1));
	gpio_set_level(PIN_SCL, 1);
	vTaskDelay(pdMS_TO_TICKS(1));
	gpio_set_level(PIN_SDA, 1);
	vTaskDelay(pdMS_TO_TICKS(1));
}

static void i2c_reinstall_driver(bool clear_bus) {
	i2c_driver_delete(0);
	if (clear_bus) {
		i2c_bus_clear();
	}

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

static bool subcommands_read16(
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

static bool bms_disable_balancing_hw(bool include_ic2) {
	bms_clear_balance_state();

	for (int i = 0; i < BMS_BALANCE_OFF_RETRIES; i++) {
		bool res1 = subcommands_write16(BQ_ADDR_1, CB_ACTIVE_CELLS, 0);
		bool res2 = true;

		if (include_ic2) {
			res2 = subcommands_write16(BQ_ADDR_2, CB_ACTIVE_CELLS, 0);
		}

		if (res1 && res2) {
			return true;
		}

		vTaskDelay(pdMS_TO_TICKS(BMS_BALANCE_OFF_RETRY_DELAY_MS));
	}

	return false;
}

static bool bms_fail_close_outputs_hw(bool include_ic2) {
	bms_set_chg_hw(false);
	return bms_disable_balancing_hw(include_ic2);
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

static uint8_t clamp_u8_i(int value, int min, int max) {
	if (value < min) {
		value = min;
	} else if (value > max) {
		value = max;
	}

	return (uint8_t)value;
}

static uint8_t overcurrent_threshold_from_a(float current) {
	// BQ76952 OCC/OCD thresholds are shunt voltage in 2 mV steps.
	float shunt_mv = current * HW_R_SHUNT * 1000.0f;
	return clamp_u8_i((int)((shunt_mv / 2.0f) + 0.5f), 2, 62);
}

static uint8_t scd_threshold_from_a(float current) {
	static const float thresholds_mv[] = {
		10.0f, 20.0f, 40.0f, 60.0f, 80.0f, 100.0f, 125.0f, 150.0f,
		175.0f, 200.0f, 250.0f, 300.0f, 350.0f, 400.0f, 450.0f, 500.0f
	};

	float target_current = fmaxf(current * 2.5f, 40.0f);
	float target_mv = target_current * HW_R_SHUNT * 1000.0f;

	for (size_t i = 0; i < (sizeof(thresholds_mv) / sizeof(thresholds_mv[0])); i++) {
		if (target_mv <= thresholds_mv[i]) {
			return (uint8_t)i;
		}
	}

	return 15;
}

static void bq_init(uint8_t dev_addr, bool current_protection_en) {
	command_subcommands(dev_addr, EXIT_DEEPSLEEP);
	command_subcommands(dev_addr, EXIT_DEEPSLEEP);
	vTaskDelay(10);

	//command_subcommands(dev_addr, BQ769x2_RESET);
	//vTaskDelay(60);

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

	// Disabled
	bq_set_reg(dev_addr, CFETOFFPinConfig, 0x00, 1);
	bq_set_reg(dev_addr, DFETOFFPinConfig, 0x00, 1);

	// ADC inputs with 18k pull-up
	main_config_t *cfg = (main_config_t *)&backup.config;
	uint32_t ntcPinConfig = 0;
	/* 0b00111011
	   00: 18k pull-up
	   11: no polynomial is used, raw ADC counts are reported
	   10: thermistor temperature measurement, reported but not used for protections
	   11: 3 = ADC Input or Thermistor
	   0b01111011
	   01: 180k pull-up
	   11: no polynomial is used, raw ADC counts are reported
	   10: thermistor temperature measurement, reported but not used for protections
	   11: 3 = ADC Input or Thermistor
	*/

	switch (cfg->temp_res) {
		case NTC_RES_4_7K:
			ntcPinConfig = 0b00111011;
			break;
		case NTC_RES_5K:
			ntcPinConfig = 0b00111011;
			break;
		case NTC_RES_10K:
			ntcPinConfig = 0b00111011;
			break;
		case NTC_RES_20K:
			ntcPinConfig = 0b00111011;
			break;
		case NTC_RES_22K:
			ntcPinConfig = 0b00111011;
			break;
		case NTC_RES_47K:
			ntcPinConfig = 0b00111011;
			break;
		case NTC_RES_50K:
			ntcPinConfig = 0b00111011;
			break;
		case NTC_RES_100K:
			ntcPinConfig = 0b01111011;
			break;
		case NTC_RES_200K:
			ntcPinConfig = 0b01111011;
			break;
	}

	bq_set_reg(dev_addr, TS1Config, ntcPinConfig, 1);
	bq_set_reg(dev_addr, TS3Config, ntcPinConfig, 1);
	bq_set_reg(dev_addr, ALERTPinConfig, ntcPinConfig, 1);
	bq_set_reg(dev_addr, DCHGPinConfig, ntcPinConfig, 1);
	bq_set_reg(dev_addr, HDQPinConfig, 0b00111011, 1);

	if (current_protection_en) {
		// DDSG is routed to the charge gate disable circuit. Configure it as an
		// active-low hardware fault output. BQ voltage protections are intentionally
		// left disabled here because only the lower BQ DDSG is wired; software
		// charge control uses both BQ ICs for cell-voltage cutoff.
		bq_set_reg(dev_addr, DDSGPinConfig, 0x82, 1);
	} else {
		bq_set_reg(dev_addr, DDSGPinConfig, 0x00, 1);
	}

	// Use all cells
	bq_set_reg(dev_addr, VCellMode, 0x0000, 2);

	// The main charge/discharge MOSFETs are driven by external hardware, not
	// the BQ high-side FET drivers. Keep the BQ charge pump off.
	bq_set_reg(dev_addr, ChgPumpControl, 0x00, 1);

	bq_set_reg(dev_addr, MfgStatusInit, 0x10, 1); // FET_EN
	if (current_protection_en) {
		// Configure BQ1 current protections. Software still owns normal charge
		// control and all voltage limits. This board routes BQ1 DDSG into the
		// charge gate disable path, so include charge-current OCC in the
		// DSG-side action mask as well.
		bq_set_reg(dev_addr, OCCThreshold, overcurrent_threshold_from_a(cfg->max_charge_current), 1);
		bq_set_reg(dev_addr, OCCDelay, 1, 1);
		bq_set_reg(dev_addr, OCD1Threshold, overcurrent_threshold_from_a(cfg->max_charge_current), 1);
		bq_set_reg(dev_addr, OCD1Delay, 1, 1);
		bq_set_reg(dev_addr, SCDThreshold, scd_threshold_from_a(cfg->max_charge_current), 1);
		bq_set_reg(dev_addr, SCDDelay, 1, 1); // no extra delay
		bq_set_reg(dev_addr, SCDLLatchLimit, 1, 1);
		// Keep TI's fast-turnoff CHG mask value. Voltage faults are disabled
		// globally below, so only the enabled current protections can act.
		bq_set_reg(dev_addr, CHGFETProtectionsA, 0x98, 1); // TI fast CHG mask
		bq_set_reg(dev_addr, DSGFETProtectionsA, 0xF4, 1); // SCD + OCD2 + OCD1 + OCC + CUV
		bq_set_reg(dev_addr, EnabledProtectionsA, 0xB0, 1); // SCD + OCD1 + OCC
	} else {
		// BQ2 has no current shunt and no DDSG charge-gate path on this board.
		bq_set_reg(dev_addr, CHGFETProtectionsA, 0x00, 1);
		bq_set_reg(dev_addr, DSGFETProtectionsA, 0x00, 1);
		bq_set_reg(dev_addr, EnabledProtectionsA, 0x00, 1);
	}
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

	command_subcommands(dev_addr, ALL_FETS_ON);
	command_subcommands(dev_addr, SLEEP_DISABLE);
}

// Extensions

// ext_bms_init - bring up the dual-BQ76952 stack on a single I2C bus.
//
// At cold boot both chips power up at the BQ76952 default address 0x08.
// To run them on the same bus we move BQ1 to 0x10, leaving BQ2 at 0x08.
// PIN_COM_EN is wired so that pulling it HIGH silences BQ2's I2C interface,
// giving us a window in which only BQ1 is reachable on the bus.
//
// Naming caveat - read carefully:
//   BQ_ADDR_1 = 0x10 - BQ1's address AFTER it has been re-addressed.
//   BQ_ADDR_2 = 0x08 - BQ2's address (always), AND BQ1's default address.
// During the address-change window below we talk to BQ1 using BQ_ADDR_2
// (= 0x08), because BQ1 hasn't moved yet and BQ2 is silenced via COM_EN.
// So `bq_init(BQ_ADDR_2)` followed by `bq_set_reg(BQ_ADDR_2, I2CAddress, ...)`
// is initialising BQ1, not BQ2. This is confusing but correct.
//
// Phases:
//   1. PIN_COM_EN = HIGH - silence BQ2.
//   2. Reinstall the I2C driver to recover from any wedged bus state.
//   3. RESET BQ1 at BQ_ADDR_1 (works on warm boot when BQ1 is at 0x10;
//      NAKs harmlessly on cold boot when BQ1 is already at 0x08). After
//      RESET, BQ1 is guaranteed to be at default 0x08 either way.
//   4. Configure BQ1 at BQ_ADDR_2 (0x08), write I2CAddress = 0x20 so that
//      EXIT_CFGUPDATE moves BQ1 to 0x10.
//   5. PIN_COM_EN = LOW - bring BQ2 back on the bus, now at 0x08 with no
//      address conflict (BQ1 is at 0x10).
//   6. Configure BQ2 at BQ_ADDR_2 (0x08).
static lbm_value ext_bms_init(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	unsigned int cells_ic1 = 16;
	if (argn >= 1) {
		cells_ic1 = lbm_dec_as_u32(args[0]);
	}

	unsigned int cells_ic2 = 16;
	if (argn >= 2) {
		cells_ic2 = lbm_dec_as_u32(args[1]);
	}

	if (cells_ic1 < 3 || cells_ic1 > 16 || cells_ic2 > 16 || cells_ic2 == 1
		|| cells_ic2 == 2) {
		lbm_set_error_reason("Invalid cell combination");
		return ENC_SYM_TERROR;
	}

	bms_set_chg_hw(false);
	bms_clear_balance_state();

	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		lbm_set_error_reason("bq_mutex timeout in bms-init");
		return ENC_SYM_NIL;
	}

	gpio_set_level(PIN_COM_EN, 0);
	(void)bms_disable_balancing_hw(m_cells_ic2 != 0 || cells_ic2 != 0);

	// i2c_tx_rx() serializes through i2c_mutex, but i2c_driver_delete /
	// _install bypass it. If any other caller (Lisp i2c-tx-rx, i2c-detect,
	// future hardware) is mid-transfer on port 0 while we tear it down, the
	// driver state goes invalid. Hold both mutexes across the reinstall.
	if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		xSemaphoreGive(bq_mutex);
		lbm_set_error_reason("i2c_mutex timeout in bms-init");
		return ENC_SYM_NIL;
	}

	// Disable COMM until the i2c-address of the first BQ
	// is changed.
	gpio_set_level(PIN_COM_EN, 1);

	// Fast path: only reinstall the ESP I2C driver. Bus-clear clocking is
	// reserved for retry recovery after a real BQ communication failure.
	i2c_reinstall_driver(false);

	xSemaphoreGive(i2c_mutex);

	vTaskDelay(50);

	// Wake BQ1 if it was put into BQ-level DEEPSLEEP by a previous
	// bms-sleep. The BQ I2C interface stays alive in DEEPSLEEP listening
	// for EXIT_DEEPSLEEP at the chip's address (which persists through
	// sleep - BQ1 stays at 0x10 once moved). Cold-boot fresh chips are
	// still at 0x08, so this NAKs harmlessly.
	command_subcommands(BQ_ADDR_1, EXIT_DEEPSLEEP);
	command_subcommands(BQ_ADDR_1, EXIT_DEEPSLEEP);
	vTaskDelay(pdMS_TO_TICKS(10));

	// Probe BQ1 at its post-init address (0x10) first. If it ACKs, the
	// address change has already been done - either from a previous boot
	// in this session or because the new I2CAddress is committed to data
	// flash and survives BQ769x2_RESET. In that case the old code path
	// (RESET -> write I2CAddress at 0x08) NAKs forever because nothing is
	// at 0x08 (BQ2 silenced via COM_EN, BQ1 still at 0x10) - which is the
	// "Could not update I2C address" loop seen on warm boots.
	//
	// COM_EN=1 here means BQ2 is silenced, so an ACK at 0x10 can only be
	// from BQ1. Use a single direct command read; no state changes if it
	// fails.
	bool bq1_at_target = false;
	command_read(BQ_ADDR_1, Cell1Voltage, &bq1_at_target);

	if (!bq1_at_target) {
		bool addr_update_ok = false;

		for (int attempt = 0; attempt < BMS_INIT_ADDR_RETRIES; attempt++) {
			if (attempt > 0) {
				if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
					i2c_reinstall_driver(true);
					xSemaphoreGive(i2c_mutex);
				}

				gpio_set_level(PIN_COM_EN, 1);
				vTaskDelay(pdMS_TO_TICKS(50));
			}

			// Cold-boot / DF-default path: BQ1 expected at default 0x08.
			// BQ76952 datasheet allows up to ~260 ms for RESET to complete; a
			// shorter delay can leave the chip half-reset and wedge the bus.
			// Do not send SWAP_COMM_MODE at 0x10 here: after RESET the chip is
			// expected at its default address, so that old call only NAKed.
			command_subcommands(BQ_ADDR_1, EXIT_DEEPSLEEP);
			command_subcommands(BQ_ADDR_1, EXIT_DEEPSLEEP);
			command_subcommands(BQ_ADDR_2, EXIT_DEEPSLEEP);
			command_subcommands(BQ_ADDR_2, EXIT_DEEPSLEEP);
			command_subcommands(BQ_ADDR_1, BQ769x2_RESET);
			vTaskDelay(pdMS_TO_TICKS(300));

			bq_init(BQ_ADDR_2, true);
			command_subcommands(BQ_ADDR_2, SET_CFGUPDATE);
			if (!bq_set_reg(BQ_ADDR_2, I2CAddress, 0x20, 1)) {
				continue;
			}

			command_subcommands(BQ_ADDR_2, EXIT_CFGUPDATE);
			// I2CAddress is applied on reset or SWAP_COMM_MODE. We are already
			// in I2C mode; this makes the new address take effect immediately.
			command_subcommands(BQ_ADDR_2, SWAP_COMM_MODE);
			vTaskDelay(pdMS_TO_TICKS(20));

			command_read(BQ_ADDR_1, Cell1Voltage, &addr_update_ok);
			if (addr_update_ok) {
				break;
			}
		}

		if (!addr_update_ok) {
			lbm_set_error_reason("BQ1 I2C address recovery failed");
			xSemaphoreGive(bq_mutex);
			return ENC_SYM_NIL;
		}
	}

	// Enable the other i2c now that BQ1's address is settled at 0x10
	gpio_set_level(PIN_COM_EN, 0);
	vTaskDelay(50);

	if (cells_ic2 != 0) {
		bq_init(BQ_ADDR_2, false);
	}

	// Always init BQ1 at its final address so its config is fresh on
	// every bms-init, regardless of which path we took above.
	bq_init(BQ_ADDR_1, true);

	m_cells_ic1 = cells_ic1;
	m_cells_ic2 = cells_ic2;

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

	// Sleep entry is rare and not latency-critical. Use a generous timeout
	// (5 s vs the 500 ms default) so transient I2C contention doesn't cause
	// us to skip BQ DEEPSLEEP - that would leave each BQ in active mode
	// (~mA) drawing only from its top cell, causing progressive top-cell
	// imbalance over weeks of 2 h-cycle wakes. Return EERROR (not NIL) so
	// Lisp `with-com` retries via its trap loop instead of silently letting
	// the caller proceed to sleep-deep with the BQs still active.
	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
		lbm_set_error_reason("bq_mutex timeout in bms-sleep");
		return ENC_SYM_EERROR;
	}

	// Pre-flight: confirm both BQs respond at their expected addresses
	// BEFORE we disable switches and start writing sleep config. If BQ1
	// somehow ended up at the default 0x08 (e.g. a previous bms-init
	// partially failed), DEEPSLEEP at 0x10 would NAK while DEEPSLEEP at
	// 0x08 might put the wrong chip to sleep - and we'd discover this
	// only after switches are off and TS pins reconfigured, leaving the
	// device in a half-prepared-for-sleep state. Bail early and let
	// Lisp's recovery path re-run init-hw to re-establish addresses.
	{
		bool probe_ok = false;
		command_read(BQ_ADDR_1, Cell1Voltage, &probe_ok);
		if (!probe_ok) {
			xSemaphoreGive(bq_mutex);
			lbm_set_error_reason("BQ1 not responsive at 0x10 before sleep");
			return ENC_SYM_EERROR;
		}
		if (m_cells_ic2 != 0) {
			probe_ok = false;
			command_read(BQ_ADDR_2, Cell1Voltage, &probe_ok);
			if (!probe_ok) {
				xSemaphoreGive(bq_mutex);
				lbm_set_error_reason("BQ2 not responsive at 0x08 before sleep");
				return ENC_SYM_EERROR;
			}
		}
	}

	// Disable all switches
	gpio_set_level(PIN_OUT_EN, 0);
	bms_set_chg_hw(false);
	gpio_set_level(PIN_PSW_EN, 0);

	// Stop balancing
	if (!bms_disable_balancing_hw(m_cells_ic2 != 0)) {
		goto exit_error1;
	}

	// Disable temperature measurement pull-ups and ensure that regulator is kept on in DEEP SLEEP

	if (!command_subcommands(BQ_ADDR_1, SET_CFGUPDATE) ||
		!bq_set_reg(BQ_ADDR_1, PowerConfig, 0b0010011010000000, 2) ||
		!bq_set_reg(BQ_ADDR_1, TS1Config, 0x00, 1) ||
		!bq_set_reg(BQ_ADDR_1, TS3Config, 0x00, 1) ||
		!command_subcommands(BQ_ADDR_1, EXIT_CFGUPDATE)) {
		goto exit_error1;
	}

	if (m_cells_ic2 != 0) {
		if (!command_subcommands(BQ_ADDR_2, SET_CFGUPDATE) ||
			!bq_set_reg(BQ_ADDR_2, PowerConfig, 0b0010011010000000, 2) ||
			!bq_set_reg(BQ_ADDR_2, TS1Config, 0x00, 1) ||
			!bq_set_reg(BQ_ADDR_2, TS3Config, 0x00, 1) ||
			!command_subcommands(BQ_ADDR_2, EXIT_CFGUPDATE)) {
				goto exit_error2;
			}
	}

	// DEEPSLEEP requires the subcommand to be sent twice in succession
	// (TI safety pattern: first call arms, second commits). Error-check
	// both: a NAK here means the chip stayed in ACTIVE mode (~mA draw
	// from the top cell) while ESP goes to deep sleep - silent battery
	// damage. Surface failures so Lisp doesn't proceed to sleep-deep.
	if (!command_subcommands(BQ_ADDR_1, DEEPSLEEP) ||
		!command_subcommands(BQ_ADDR_1, DEEPSLEEP)) {
		goto exit_error1;
	}

	if (m_cells_ic2 != 0) {
		if (!command_subcommands(BQ_ADDR_2, DEEPSLEEP) ||
			!command_subcommands(BQ_ADDR_2, DEEPSLEEP)) {
			goto exit_error2;
		}
	}

	// Disable CAN-bus and other COMM
	gpio_set_level(PIN_COM_EN, 1);

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

static bool bms_shutdown_bq(uint8_t addr) {
	// SHUTDOWN requires the subcommand to be sent twice in succession.
	return command_subcommands(addr, SHUTDOWN) &&
			command_subcommands(addr, SHUTDOWN);
}

static void bms_config_shutdown_wakeup(void) {
	gpio_set_direction(PIN_ENABLE, GPIO_MODE_INPUT);

#if CONFIG_IDF_TARGET_ESP32S3
	esp_sleep_enable_ext0_wakeup(PIN_ENABLE, 1);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
#elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C6
	esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(
			1ULL << PIN_ENABLE, ESP_GPIO_WAKEUP_GPIO_HIGH);
#else
#error "Unsupported target"
#endif
}

static lbm_value ext_bms_hw_shutdown(lbm_value *args, lbm_uint argn) {
	(void)args;

	if (argn != 0) {
		return ENC_SYM_TERROR;
	}

	bms_set_chg_hw(false);
	bms_clear_balance_state();

	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
		lbm_set_error_reason("bq_mutex timeout in bms-hw-shutdown");
		return ENC_SYM_EERROR;
	}

	gpio_set_level(PIN_COM_EN, 0);
	(void)bms_disable_balancing_hw(m_cells_ic2 != 0);

	if (m_cells_ic2 != 0 && !bms_shutdown_bq(BQ_ADDR_2)) {
		(void)bms_fail_close_outputs_hw(m_cells_ic2 != 0);
		xSemaphoreGive(bq_mutex);
		lbm_set_error_reason("BQ2 shutdown command failed");
		return ENC_SYM_EERROR;
	}

	if (!bms_shutdown_bq(BQ_ADDR_1)) {
		(void)bms_fail_close_outputs_hw(m_cells_ic2 != 0);
		xSemaphoreGive(bq_mutex);
		lbm_set_error_reason("BQ1 shutdown command failed");
		return ENC_SYM_EERROR;
	}

	xSemaphoreGive(bq_mutex);

	vTaskDelay(pdMS_TO_TICKS(1000));

#ifdef SHUTDOWN_SUPPORT
#if !SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP
	gpio_deep_sleep_hold_dis();
#endif
	// Disable ESP GPIO hold so the open-drain shutdown output can change
	// from idle/released high to asserted low.
	gpio_hold_dis(PIN_SHUTDOWN);
	gpio_set_level(PIN_SHUTDOWN, 0);

	vTaskDelay(pdMS_TO_TICKS(10000));
	bms_set_chg_hw(false);
	bms_clear_balance_state();
	lbm_set_error_reason("BMS shutdown pin did not power off");
	return ENC_SYM_EERROR;
#else
	bms_config_shutdown_wakeup();
	gpio_set_level(PIN_COM_EN, 1);
	esp_deep_sleep_start();

	return ENC_SYM_TRUE;
#endif
}

static lbm_value ext_get_vcells(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	lbm_value vc_list = ENC_SYM_NIL;

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

	return lbm_list_destructive_reverse(vc_list);
}

static lbm_value ext_cell0_report_offset(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	float offset = 0.0;
#ifndef SHUTDOWN_SUPPORT
	float current = lbm_dec_as_float(args[0]);
	if (current < 0.0) {
		offset = 0.003 * fabsf(current);
		utils_truncate_number(&offset, 0.0, 0.015);
	}
#endif

	return lbm_enc_float(offset);
}

#define NTC_TEMP(res, ntc_res, ntc_beta) \
	(1.0 / ((logf((res) / ntc_res) / ntc_beta) + (1.0 / 298.15)) - 273.15)
#define NAN_TO_M1(x)   (UTILS_IS_NAN(x) ? -1.0 : x)

static float ntc_measured_res(float volts, float pullup_res) {
	if (volts <= 0.0 || volts >= 1.79) {
		return NAN;
	}

	return pullup_res / (1.8 / volts - 1.0) - 500.0;
}

static lbm_value ext_get_temps(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	lbm_value ts_list = ENC_SYM_NIL;
	bool ok           = false;
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

	float v1 = (float)command_read(BQ_ADDR_1, TS1Temperature, &ok)
		* counts_to_volts;
	if (!ok) {
		goto exit_error1;
	}
	float v2 = (float)command_read(BQ_ADDR_1, TS3Temperature, &ok)
		* counts_to_volts;
	if (!ok) {
		goto exit_error1;
	}
	float v3 = (float)command_read(BQ_ADDR_1, ALERTTemperature, &ok)
		* counts_to_volts;
	if (!ok) {
		goto exit_error1;
	}
	float v4 = (float)command_read(BQ_ADDR_1, DCHGTemperature, &ok)
		* counts_to_volts;
	if (!ok) {
		goto exit_error1;
	}
	float v5 = (float)command_read(BQ_ADDR_1, HDQTemperature, &ok)
		* counts_to_volts;
	if (!ok) {
		goto exit_error1;
	}

	float ntc_smd_beta = 3434; // B25/85 LCSC=C13564 muRata NCP18XH103F03RB
	float ntc_smd_res = 10000.0;
	float ntc_pullup_res = 18000.0;

	main_config_t *cfg = (main_config_t *)&backup.config;
	float ntc_res = 0.0;

	switch (cfg->temp_res) {
		case NTC_RES_4_7K:
			ntc_res = 4700.0;
			break;
		case NTC_RES_5K:
			ntc_res = 5000.0;
			break;
		case NTC_RES_10K:
			ntc_res = 10000.0;
			break;
		case NTC_RES_20K:
			ntc_res = 20000.0;
			break;
		case NTC_RES_22K:
			ntc_res = 22000.0;
			break;
		case NTC_RES_47K:
			ntc_res = 47000.0;
			break;
		case NTC_RES_50K:
			ntc_res = 50000.0;
			break;
		case NTC_RES_100K:
			ntc_res = 100000.0;
			ntc_pullup_res = 180000.0;
			break;
		case NTC_RES_200K:
			ntc_res = 200000.0;
			ntc_pullup_res = 180000.0;
			break;
	}

	float beta = (float)(cfg->temp_beta);
	ts_list = lbm_cons(lbm_enc_float(NAN_TO_M1(
			NTC_TEMP(ntc_measured_res(v1, ntc_pullup_res), ntc_res, beta))), ts_list);
	ts_list = lbm_cons(lbm_enc_float(NAN_TO_M1(
			NTC_TEMP(ntc_measured_res(v2, ntc_pullup_res), ntc_res, beta))), ts_list);
	ts_list = lbm_cons(lbm_enc_float(NAN_TO_M1(
			NTC_TEMP(ntc_measured_res(v3, ntc_pullup_res), ntc_res, beta))), ts_list);
	ts_list = lbm_cons(lbm_enc_float(NAN_TO_M1(
			NTC_TEMP(ntc_measured_res(v4, ntc_pullup_res), ntc_res, beta))), ts_list);
	ts_list = lbm_cons(lbm_enc_float(NAN_TO_M1(
			NTC_TEMP(ntc_measured_res(v5, 18000.0), ntc_smd_res, ntc_smd_beta))), ts_list);

	if (m_cells_ic2 != 0) {
		ts_list = lbm_cons(
			lbm_enc_float(
				(float)command_read(BQ_ADDR_2, IntTemperature, &ok) * 0.1
				- 273.15
			),
			ts_list
		);
		if (!ok) {
			goto exit_error2;
		}

		// JFBMS32 uses the HDQ pin thermistors for MOSFET temperature:
		// BQ1 HDQ = first MOSFET NTC, BQ2 HDQ = second MOSFET NTC.
		float v6 = (float)command_read(BQ_ADDR_2, HDQTemperature, &ok)
			* counts_to_volts;
		if (!ok) {
			goto exit_error2;
		}

		ts_list = lbm_cons(lbm_enc_float(NAN_TO_M1(
				NTC_TEMP(ntc_measured_res(v6, 18000.0), ntc_smd_res, ntc_smd_beta))), ts_list);
	} else {
		ts_list = lbm_cons(lbm_enc_float(-1.0), ts_list);
		ts_list = lbm_cons(lbm_enc_float(-1.0), ts_list);
	}

	return lbm_list_destructive_reverse(ts_list);

exit_error1:
	lbm_set_error_reason(error_comm_bq1);
	return ENC_SYM_EERROR;

exit_error2:
	lbm_set_error_reason(error_comm_bq2);
	return ENC_SYM_EERROR;
}

static lbm_value ext_get_current(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	bool ok       = false;
	float current = ((float)command_read(BQ_ADDR_1, CC2Current, &ok) / 100.0);

	if (!ok) {
		lbm_set_error_reason(error_comm_bq1);
		return ENC_SYM_EERROR;
	}

	return lbm_enc_float(current);
}

static lbm_value ext_get_vout(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_float(HW_GET_VOUT());
}

static lbm_value ext_get_vchg(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_float(HW_GET_VCHG());
}

static lbm_value ext_bms_supports_shutdown(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	// Both JFBMS32 variants support the Lisp shutdown command. v2 asserts
	// PIN_SHUTDOWN; v1 shuts down the BQs and then enters ESP deep sleep.
	return ENC_SYM_TRUE;
}

static lbm_value ext_get_time_of_day_s(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	struct timeval now;
	gettimeofday(&now, NULL);

	return lbm_enc_i32(now.tv_sec);
}

// Returns 1 for wakeup source GPIO IO or RTC IO, 2 for timer and 0 for any other source
static lbm_value ext_bms_wakeup_source(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	uint32_t wakeup_causes = esp_sleep_get_wakeup_causes();

	if (wakeup_causes & (BIT(ESP_SLEEP_WAKEUP_EXT0) | BIT(ESP_SLEEP_WAKEUP_GPIO))) {
		return lbm_enc_i(1);
	}

	if (wakeup_causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) {
		return lbm_enc_i(2);
	}

	return lbm_enc_i(0);
}

static lbm_value ext_get_btn(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_i(gpio_get_level(PIN_ENABLE) == 0 ? 0 : 1);
}

static lbm_value ext_set_btn_wakeup_state(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	switch (lbm_dec_as_i32(args[0])) {
		case 0:
			esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(
				1ULL << PIN_ENABLE, ESP_GPIO_WAKEUP_GPIO_LOW
			);
			break;

		case 1:
			esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(
				1ULL << PIN_ENABLE, ESP_GPIO_WAKEUP_GPIO_HIGH
			);
			break;

		default:
			gpio_wakeup_disable_on_hp_periph_powerdown_sleep(PIN_ENABLE);
			break;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_set_out(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	gpio_set_level(PIN_PSW_EN, 1);
	gpio_set_level(PIN_OUT_EN, lbm_dec_as_i32(args[0]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_set_chg(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	bms_set_chg_hw(lbm_dec_as_i32(args[0]) != 0);
	return ENC_SYM_TRUE;
}

static lbm_value ext_bms_disable_balancing(lbm_value *args, lbm_uint argn) {
	(void)args;

	if (argn != 0) {
		return ENC_SYM_TERROR;
	}

	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		bms_clear_balance_state();
		lbm_set_error_reason("bq_mutex timeout in bms-disable-balancing");
		return ENC_SYM_EERROR;
	}

	int com_prev = gpio_get_level(PIN_COM_EN);
	gpio_set_level(PIN_COM_EN, 0);
	bool res = bms_disable_balancing_hw(m_cells_ic2 != 0);
	gpio_set_level(PIN_COM_EN, com_prev);
	xSemaphoreGive(bq_mutex);

	if (!res) {
		lbm_set_error_reason("BMS balancing disable failed");
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_bms_fail_close_outputs(lbm_value *args, lbm_uint argn) {
	(void)args;

	if (argn != 0) {
		return ENC_SYM_TERROR;
	}

	bms_set_chg_hw(false);

	if (xSemaphoreTake(bq_mutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
		bms_clear_balance_state();
		lbm_set_error_reason("bq_mutex timeout in bms-fail-close-outputs");
		return ENC_SYM_EERROR;
	}

	int com_prev = gpio_get_level(PIN_COM_EN);
	gpio_set_level(PIN_COM_EN, 0);
	bool res = bms_fail_close_outputs_hw(m_cells_ic2 != 0);
	gpio_set_level(PIN_COM_EN, com_prev);
	xSemaphoreGive(bq_mutex);

	if (!res) {
		lbm_set_error_reason("BMS fail-close balance disable failed");
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_set_bal(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	unsigned int ch = lbm_dec_as_u32(args[0]);
	int state       = lbm_dec_as_i32(args[1]);
	bool res        = false;

	if (ch < m_cells_ic1) {
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
		if (state) {
			m_bal_state_ic2 |= (1 << (ch - m_cells_ic1));
		} else {
			m_bal_state_ic2 &= ~(1 << (ch - m_cells_ic1));
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

	// Read from IC
	//	if (ch < m_cells_ic1) {
	//		uint16_t state;
	//		if (subcommands_read16(BQ_ADDR_1, CB_ACTIVE_CELLS, &state)) {
	//			res = (state >> ch) & 0x01;
	//		}
	//	} else if ((ch - m_cells_ic1) < m_cells_ic2) {
	//		uint16_t state;
	//		if (subcommands_read16(BQ_ADDR_2, CB_ACTIVE_CELLS, &state)) {
	//			res = (state >> (ch - m_cells_ic1)) & 0x01;
	//		}
	//	}

	(void)subcommands_read16;

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
	lbm_uint cells_ic1;
	lbm_uint cells_ic2;
	lbm_uint temp_num;
	lbm_uint batt_ah;
	lbm_uint max_bal_ch;
	lbm_uint soc_use_ah;
	lbm_uint block_sleep;
	lbm_uint vc_empty;
	lbm_uint vc_full;
	lbm_uint vc_balance_start;
	lbm_uint vc_balance_end;
	lbm_uint vc_charge_start;
	lbm_uint vc_charge_end;
	lbm_uint vc_charge_min;
	lbm_uint vc_balance_min;
	lbm_uint balance_max_current;
	lbm_uint min_current_ah_wh_cnt;
	lbm_uint min_current_sleep;
	lbm_uint v_charge_detect;
	lbm_uint t_charge_max;
	lbm_uint t_charge_max_mos;
	lbm_uint sleep;
	lbm_uint min_charge_current;
	lbm_uint max_charge_current;
	lbm_uint soc_filter_const;
	lbm_uint t_bal_max_cell;
	lbm_uint t_bal_max_ic;
	lbm_uint t_charge_min;
	lbm_uint t_charge_mon_en;
	lbm_uint temp_beta;
	lbm_uint temp_res;
	lbm_uint shutdown;
} vesc_syms;

static vesc_syms syms_vesc = {0};

static bool compare_symbol(lbm_uint sym, lbm_uint *comp) {
	if (*comp == 0) {
		if (comp == &syms_vesc.cells_ic1) {
			lbm_add_symbol_const("cells_ic1", comp);
		} else if (comp == &syms_vesc.cells_ic2) {
			lbm_add_symbol_const("cells_ic2", comp);
		} else if (comp == &syms_vesc.temp_num) {
			lbm_add_symbol_const("temp_num", comp);
		} else if (comp == &syms_vesc.batt_ah) {
			lbm_add_symbol_const("batt_ah", comp);
		} else if (comp == &syms_vesc.max_bal_ch) {
			lbm_add_symbol_const("max_bal_ch", comp);
		} else if (comp == &syms_vesc.soc_use_ah) {
			lbm_add_symbol_const("soc_use_ah", comp);
		} else if (comp == &syms_vesc.block_sleep) {
			lbm_add_symbol_const("block_sleep", comp);
		} else if (comp == &syms_vesc.vc_empty) {
			lbm_add_symbol_const("vc_empty", comp);
		} else if (comp == &syms_vesc.vc_full) {
			lbm_add_symbol_const("vc_full", comp);
		} else if (comp == &syms_vesc.vc_balance_start) {
			lbm_add_symbol_const("vc_balance_start", comp);
		} else if (comp == &syms_vesc.vc_balance_end) {
			lbm_add_symbol_const("vc_balance_end", comp);
		} else if (comp == &syms_vesc.vc_charge_start) {
			lbm_add_symbol_const("vc_charge_start", comp);
		} else if (comp == &syms_vesc.vc_charge_end) {
			lbm_add_symbol_const("vc_charge_end", comp);
		} else if (comp == &syms_vesc.vc_charge_min) {
			lbm_add_symbol_const("vc_charge_min", comp);
		} else if (comp == &syms_vesc.vc_balance_min) {
			lbm_add_symbol_const("vc_balance_min", comp);
		} else if (comp == &syms_vesc.balance_max_current) {
			lbm_add_symbol_const("balance_max_current", comp);
		} else if (comp == &syms_vesc.min_current_ah_wh_cnt) {
			lbm_add_symbol_const("min_current_ah_wh_cnt", comp);
		} else if (comp == &syms_vesc.min_current_sleep) {
			lbm_add_symbol_const("min_current_sleep", comp);
		} else if (comp == &syms_vesc.v_charge_detect) {
			lbm_add_symbol_const("v_charge_detect", comp);
		} else if (comp == &syms_vesc.t_charge_max) {
			lbm_add_symbol_const("t_charge_max", comp);
		} else if (comp == &syms_vesc.t_charge_max_mos) {
			lbm_add_symbol_const("t_charge_max_mos", comp);
		} else if (comp == &syms_vesc.sleep) {
			lbm_add_symbol_const("sleep", comp);
		} else if (comp == &syms_vesc.shutdown) {
			lbm_add_symbol_const("shutdown", comp);
		} else if (comp == &syms_vesc.min_charge_current) {
			lbm_add_symbol_const("min_charge_current", comp);
		} else if (comp == &syms_vesc.max_charge_current) {
			lbm_add_symbol_const("max_charge_current", comp);
		} else if (comp == &syms_vesc.soc_filter_const) {
			lbm_add_symbol_const("soc_filter_const", comp);
		} else if (comp == &syms_vesc.t_bal_max_cell) {
			lbm_add_symbol_const("t_bal_max_cell", comp);
		} else if (comp == &syms_vesc.t_bal_max_ic) {
			lbm_add_symbol_const("t_bal_max_ic", comp);
		} else if (comp == &syms_vesc.t_charge_min) {
			lbm_add_symbol_const("t_charge_min", comp);
		} else if (comp == &syms_vesc.t_charge_mon_en) {
			lbm_add_symbol_const("t_charge_mon_en", comp);
		} else if (comp == &syms_vesc.temp_beta) {
			lbm_add_symbol_const("temp_beta", comp);
		} else if (comp == &syms_vesc.temp_res) {
			lbm_add_symbol_const("temp_res", comp);
		}
	}

	return *comp == sym;
}

static lbm_value get_or_set_float(bool set, float *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_float(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_float(*val);
	}
}

static lbm_value get_or_set_i(bool set, int *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_i32(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_i(*val);
	}
}

static lbm_value get_or_set_u16(bool set, uint16_t *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_i32(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_i(*val);
	}
}

static lbm_value get_or_set_bool(bool set, bool *val, lbm_value *lbm_val) {
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

	if (compare_symbol(name, &syms_vesc.cells_ic1)) {
		res = get_or_set_i(set, &cfg->cells_ic1, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.cells_ic2)) {
		res = get_or_set_i(set, &cfg->cells_ic2, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.temp_num)) {
		res = get_or_set_i(set, &cfg->temp_num, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.batt_ah)) {
		res = get_or_set_float(set, &cfg->batt_ah, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.max_bal_ch)) {
		res = get_or_set_i(set, &cfg->max_bal_ch, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.soc_use_ah)) {
		res = get_or_set_bool(set, &cfg->soc_use_ah, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.block_sleep)) {
		res = get_or_set_bool(set, &cfg->block_sleep, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_empty)) {
		res = get_or_set_float(set, &cfg->vc_empty, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_full)) {
		res = get_or_set_float(set, &cfg->vc_full, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_balance_start)) {
		res = get_or_set_float(set, &cfg->vc_balance_start, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_balance_end)) {
		res = get_or_set_float(set, &cfg->vc_balance_end, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_charge_start)) {
		res = get_or_set_float(set, &cfg->vc_charge_start, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_charge_end)) {
		res = get_or_set_float(set, &cfg->vc_charge_end, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_charge_min)) {
		res = get_or_set_float(set, &cfg->vc_charge_min, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_balance_min)) {
		res = get_or_set_float(set, &cfg->vc_balance_min, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.balance_max_current)) {
		res = get_or_set_float(set, &cfg->balance_max_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.min_current_ah_wh_cnt)) {
		res = get_or_set_float(set, &cfg->min_current_ah_wh_cnt, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.min_current_sleep)) {
		res = get_or_set_float(set, &cfg->min_current_sleep, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.v_charge_detect)) {
		res = get_or_set_float(set, &cfg->v_charge_detect, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_max)) {
		res = get_or_set_float(set, &cfg->t_charge_max, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_max_mos)) {
		res = get_or_set_float(set, &cfg->t_charge_max_mos, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.sleep)) {
		res = get_or_set_float(set, &cfg->sleep, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.shutdown)) {
		res = get_or_set_u16(set, &cfg->shutdown, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.min_charge_current)) {
		res = get_or_set_float(set, &cfg->min_charge_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.max_charge_current)) {
		res = get_or_set_float(set, &cfg->max_charge_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.soc_filter_const)) {
		res = get_or_set_float(set, &cfg->soc_filter_const, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_bal_max_cell)) {
		res = get_or_set_float(set, &cfg->t_bal_max_cell, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_bal_max_ic)) {
		res = get_or_set_float(set, &cfg->t_bal_max_ic, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_min)) {
		res = get_or_set_float(set, &cfg->t_charge_min, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_mon_en)) {
		res = get_or_set_bool(set, &cfg->t_charge_mon_en, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.temp_res)) {
		res = get_or_set_i(set, (int *)(&cfg->temp_res), &set_arg);
	} else if (compare_symbol(name, &syms_vesc.temp_beta)) {
		res = get_or_set_u16(set, &cfg->temp_beta, &set_arg);
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
	return lbm_enc_i(8);
}

static void load_extensions(bool main_found) {
	if (main_found) {
		return;
	}

	memset(&syms_vesc, 0, sizeof(syms_vesc));

	// Wake up and initialize hardware
	lbm_add_extension("bms-init", ext_bms_init);

	// Put BMS hardware in sleep mode
	lbm_add_extension("bms-sleep", ext_hw_sleep);

	// Get list of cell voltages
	lbm_add_extension("bms-get-vcells", ext_get_vcells);

	// Apply HW-specific Cell 0 voltage compensation before publishing BMS data
	lbm_add_extension("bms-cell0-report-offset", ext_cell0_report_offset);

	// Get list of temperature readings
	lbm_add_extension("bms-get-temps", ext_get_temps);

	// Get current in/out. Negative numbers mean charging
	lbm_add_extension("bms-get-current", ext_get_current);

	// Get output voltage after power switch
	lbm_add_extension("bms-get-vout", ext_get_vout);

	// Get charge input voltage
	lbm_add_extension("bms-get-vchg", ext_get_vchg);

	// Get user button state
	lbm_add_extension("bms-get-btn", ext_get_btn);

	// Enable user button wakeup. 1: wakeup on ON, 0: wakeup on OFF, otherwise disable wakeup
	lbm_add_extension("bms-set-btn-wakeup-state", ext_set_btn_wakeup_state);

	//Returns 1 for wakeup source GPIO IO, 2 for timer and 0 for any other source
	lbm_add_extension("bms-wakeup-source", ext_bms_wakeup_source);

	lbm_add_extension("get-time-of-day-s", ext_get_time_of_day_s);

	lbm_add_extension("bms-supports-shutdown", ext_bms_supports_shutdown);

	lbm_add_extension("bms-hw-shutdown", ext_bms_hw_shutdown);

	lbm_add_extension("bms-disable-balancing", ext_bms_disable_balancing);
	lbm_add_extension("bms-fail-close-outputs", ext_bms_fail_close_outputs);

	// Enable/disable output switch
	lbm_add_extension("bms-set-out", ext_set_out);

	// Enable/disable charge switch
	lbm_add_extension("bms-set-chg", ext_set_chg);

	// Set and get balancing state for cell
	lbm_add_extension("bms-set-bal", ext_set_bal);
	lbm_add_extension("bms-get-bal", ext_get_bal);

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

	gpio_config_t gpconf = {0};

	gpio_set_level(PIN_OUT_EN, 0);
	gpio_set_level(PIN_CHG_EN, 0);
	gpio_set_level(PIN_SHUTDOWN, 1);
	gpio_set_level(PIN_PSW_EN, 1);
	gpio_set_level(PIN_COM_EN, 1);

	gpconf.pin_bit_mask = BIT(PIN_OUT_EN) | BIT(PIN_CHG_EN) | BIT(PIN_COM_EN);
	gpconf.intr_type    = GPIO_FLOATING;
	gpconf.mode         = GPIO_MODE_INPUT_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en   = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);

	// PIN_PSW_EN configured as plain output
	gpio_set_intr_type(PIN_PSW_EN, GPIO_INTR_DISABLE);
	gpio_set_direction(PIN_PSW_EN, GPIO_MODE_OUTPUT);
	gpio_set_pull_mode(PIN_PSW_EN, GPIO_FLOATING);

	gpio_set_level(PIN_OUT_EN, 0);
	gpio_set_level(PIN_CHG_EN, 0);

	// PIN_SHUTDOWN as open-drain output, idle high
	gpio_set_intr_type(PIN_SHUTDOWN, GPIO_INTR_DISABLE);
	gpio_set_direction(PIN_SHUTDOWN, GPIO_MODE_OUTPUT_OD);
	gpio_set_pull_mode(PIN_SHUTDOWN, GPIO_FLOATING);
	gpio_set_level(PIN_SHUTDOWN, 1);

#ifdef SHUTDOWN_SUPPORT
	// Keep DCDC enabled while the ESP32 is in deep sleep.
	gpio_hold_en(PIN_SHUTDOWN);
	gpio_deep_sleep_hold_en();
#endif

	gpio_set_level(PIN_PSW_EN, 1);
	gpio_set_level(PIN_COM_EN, 1);

	gpconf.pin_bit_mask = BIT(PIN_ENABLE);
	gpconf.intr_type    = GPIO_FLOATING;
	gpconf.mode         = GPIO_MODE_INPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en   = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);

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
