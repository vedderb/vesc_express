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

#include "hw_vbms32.h"
#include "bq769x2_defs.h"

#include "main.h"
#include "driver/i2c.h"
#include "esp_sleep.h"
#include "lispif.h"
#include "lispbm.h"
#include "commands.h"
#include "utils.h"

#include <math.h>

// Settings
#define BQ_ADDR_1 0x10
#define BQ_ADDR_2 0x08
#define I2C_SPEED 100000

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

static esp_err_t i2c_tx_rx(
	uint8_t addr, const uint8_t *write_buffer, size_t write_size,
	uint8_t *read_buffer, size_t read_size
) {

	xSemaphoreTake(i2c_mutex, portMAX_DELAY);

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
	bq_set_reg(dev_addr, TS1Config, 0b00111011, 1);
	bq_set_reg(dev_addr, TS3Config, 0b00111011, 1);
	bq_set_reg(dev_addr, ALERTPinConfig, 0b00111011, 1);
	bq_set_reg(dev_addr, DCHGPinConfig, 0b00111011, 1);
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

	unsigned int cells_ic2 = 16;
	if (argn >= 2) {
		cells_ic2 = lbm_dec_as_u32(args[1]);
	}

	if (cells_ic1 < 3 || cells_ic1 > 16 || cells_ic2 > 16 || cells_ic2 == 1
		|| cells_ic2 == 2) {
		lbm_set_error_reason("Invalid cell combination");
		return ENC_SYM_TERROR;
	}

	xSemaphoreTake(bq_mutex, portMAX_DELAY);

	// Disable COMM unil the i2c-address of the first BQ
	// is changed.
	gpio_set_level(PIN_COM_EN, 1);

	// Restart i2c

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

	vTaskDelay(50);

	// Reset the address of the first BQ just in case
	command_subcommands(BQ_ADDR_1, BQ769x2_RESET);
	vTaskDelay(60);
	command_subcommands(BQ_ADDR_1, SWAP_COMM_MODE);

	bq_init(BQ_ADDR_2);
	command_subcommands(BQ_ADDR_2, SET_CFGUPDATE);
	if (!bq_set_reg(BQ_ADDR_2, I2CAddress, 0x20, 1)) {
		commands_printf_lisp("Could not update I2C address");
	}
	command_subcommands(BQ_ADDR_2, EXIT_CFGUPDATE);
	command_subcommands(BQ_ADDR_2, SWAP_COMM_MODE);

	// Enable the other i2c now that the first address is updated
	gpio_set_level(PIN_COM_EN, 0);
	vTaskDelay(50);

	if (cells_ic2 != 0) {
		bq_init(BQ_ADDR_2);
	}

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

	xSemaphoreTake(bq_mutex, portMAX_DELAY);

	// Disable all switches
	gpio_set_level(PIN_OUT_EN, 0);
	gpio_set_level(PIN_CHG_EN, 0);
	gpio_set_level(PIN_PCHG_EN, 0);
	gpio_set_level(PIN_PSW_EN, 0);

	// Stop balancing
	m_bal_state_ic1 = 0;
	m_bal_state_ic2 = 0;

	if (!subcommands_write16(BQ_ADDR_1, CB_ACTIVE_CELLS, m_bal_state_ic1)) {
		goto exit_error1;
	}

	if (m_cells_ic2 != 0) {
		if (!subcommands_write16(BQ_ADDR_2, CB_ACTIVE_CELLS, m_bal_state_ic2)) {
			goto exit_error2;
		}
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

	command_subcommands(BQ_ADDR_1, DEEPSLEEP);
	command_subcommands(BQ_ADDR_1, DEEPSLEEP);

	if (m_cells_ic2 != 0) {
		command_subcommands(BQ_ADDR_2, DEEPSLEEP);
		command_subcommands(BQ_ADDR_2, DEEPSLEEP);
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

#define NTC_TEMP(res, beta)                                                    \
	(1.0 / ((logf((res) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)
#define NTC_RES(volts) (18.0e3 / (1.8 / volts - 1.0) - 500.0)
#define NAN_TO_M1(x)   (UTILS_IS_NAN(x) ? -1.0 : x)

static lbm_value ext_get_temps(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	lbm_value ts_list = ENC_SYM_NIL;
	bool ok           = false;
	ts_list           = lbm_cons(
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

	// TODO: Use config
	float ntc_beta = 3380.0;

	ts_list = lbm_cons(
		lbm_enc_float(NAN_TO_M1(NTC_TEMP(NTC_RES(v1), ntc_beta))), ts_list
	);
	ts_list = lbm_cons(
		lbm_enc_float(NAN_TO_M1(NTC_TEMP(NTC_RES(v2), ntc_beta))), ts_list
	);
	ts_list = lbm_cons(
		lbm_enc_float(NAN_TO_M1(NTC_TEMP(NTC_RES(v3), ntc_beta))), ts_list
	);
	ts_list = lbm_cons(
		lbm_enc_float(NAN_TO_M1(NTC_TEMP(NTC_RES(v4), ntc_beta))), ts_list
	);
	ts_list = lbm_cons(
		lbm_enc_float(NAN_TO_M1(NTC_TEMP(NTC_RES(v5), ntc_beta))), ts_list
	);

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
	} else {
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

#if PCB_VERSION == 2
	return lbm_enc_float(current);
#else
	return lbm_enc_float(-current);
#endif
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

static lbm_value ext_get_btn(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return lbm_enc_i(gpio_get_level(PIN_ENABLE) == 0 ? 0 : 1);
}

static lbm_value ext_set_btn_wakeup_state(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	switch (lbm_dec_as_i32(args[0])) {
		case 0:
			esp_deep_sleep_enable_gpio_wakeup(
				1 << PIN_ENABLE, ESP_GPIO_WAKEUP_GPIO_LOW
			);
			break;

		case 1:
			esp_deep_sleep_enable_gpio_wakeup(
				1 << PIN_ENABLE, ESP_GPIO_WAKEUP_GPIO_HIGH
			);
			break;

		default:
			gpio_deep_sleep_wakeup_disable(PIN_ENABLE);
			break;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_set_pchg(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	gpio_set_level(PIN_PSW_EN, 1);
	gpio_set_level(PIN_PCHG_EN, lbm_dec_as_i32(args[0]));
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
	gpio_set_level(PIN_PSW_EN, 1);
	gpio_set_level(PIN_CHG_EN, lbm_dec_as_i32(args[0]));
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
			lbm_set_error_reason(error_comm_bq1);
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
	lbm_uint sleep_regular;
	lbm_uint sleep_long;
	lbm_uint min_charge_current;
	lbm_uint max_charge_current;
	lbm_uint soc_filter_const;
	lbm_uint t_bal_max_cell;
	lbm_uint t_bal_max_ic;
	lbm_uint t_charge_min;
	lbm_uint t_charge_mon_en;
	lbm_uint psw_t_pchg;
	lbm_uint psw_scd_en;
	lbm_uint psw_scd_tres;
	lbm_uint t_psw_en;
	lbm_uint t_psw_max_mos;
	lbm_uint psw_wait_init;
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
		} else if (comp == &syms_vesc.sleep_regular) {
			lbm_add_symbol_const("sleep_regular", comp);
		} else if (comp == &syms_vesc.sleep_long) {
			lbm_add_symbol_const("sleep_long", comp);
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
		} else if (comp == &syms_vesc.t_charge_mon_en) {
			lbm_add_symbol_const("t_charge_mon_en", comp);
		} else if (comp == &syms_vesc.psw_t_pchg) {
			lbm_add_symbol_const("psw_t_pchg", comp);
		} else if (comp == &syms_vesc.psw_scd_en) {
			lbm_add_symbol_const("psw_scd_en", comp);
		} else if (comp == &syms_vesc.psw_scd_tres) {
			lbm_add_symbol_const("psw_scd_tres", comp);
		} else if (comp == &syms_vesc.t_psw_en) {
			lbm_add_symbol_const("t_psw_en", comp);
		} else if (comp == &syms_vesc.t_psw_max_mos) {
			lbm_add_symbol_const("t_psw_max_mos", comp);
		} else if (comp == &syms_vesc.psw_wait_init) {
			lbm_add_symbol_const("psw_wait_init", comp);
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
	} else if (compare_symbol(name, &syms_vesc.sleep_regular)) {
		res = get_or_set_float(set, &cfg->sleep_regular, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.sleep_long)) {
		res = get_or_set_float(set, &cfg->sleep_long, &set_arg);
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
	} else if (compare_symbol(name, &syms_vesc.psw_t_pchg)) {
		res = get_or_set_float(set, &cfg->psw_t_pchg, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.psw_scd_en)) {
		res = get_or_set_bool(set, &cfg->psw_scd_en, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.psw_scd_tres)) {
		res = get_or_set_i(set, &cfg->psw_scd_tres, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_psw_en)) {
		res = get_or_set_bool(set, &cfg->t_psw_en, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_psw_max_mos)) {
		res = get_or_set_float(set, &cfg->t_psw_max_mos, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.psw_wait_init)) {
		res = get_or_set_bool(set, &cfg->psw_wait_init, &set_arg);
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
	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
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

	// Enable/disable precharge switch
	lbm_add_extension("bms-set-pchg", ext_set_pchg);

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
	gpio_set_level(PIN_PCHG_EN, 0);
	gpio_set_level(PIN_PSW_EN, 0);
	gpio_set_level(PIN_COM_EN, 1);

	gpconf.pin_bit_mask = BIT(PIN_OUT_EN) | BIT(PIN_CHG_EN) | BIT(PIN_PCHG_EN)
		| BIT(PIN_COM_EN) | BIT(PIN_PSW_EN);
	gpconf.intr_type    = GPIO_FLOATING;
	gpconf.mode         = GPIO_MODE_INPUT_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en   = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);

	gpio_set_level(PIN_OUT_EN, 0);
	gpio_set_level(PIN_CHG_EN, 0);
	gpio_set_level(PIN_PCHG_EN, 0);
	gpio_set_level(PIN_PSW_EN, 0);
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
