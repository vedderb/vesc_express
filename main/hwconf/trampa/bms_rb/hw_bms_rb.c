/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se

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

#include "hw_bms_rb.h"
#include "main.h"
#include "driver/i2c.h"
#include "esp_sleep.h"
#include "lispif.h"
#include "lispbm.h"
#include "commands.h"

#include <math.h>

// Settings
#define BQ_ADDR					0x08
#define CRC_KEY					7

// Registers
#define REG_SYS_STAT			0x00
#define REG_CELLBAL1			0x01
#define REG_CELLBAL2			0x02
#define REG_CELLBAL3			0x03
#define REG_SYS_CTRL1			0x04
#define REG_SYS_CTRL2			0x05
#define REG_PROTECT1			0x06
#define REG_PROTECT2			0x07
#define REG_PROTECT3			0x08
#define REG_OV_TRIP				0x09
#define REG_UV_TRIP				0x0A
#define REG_CC_CFG				0x0B
#define REG_VC1_HI_BYTE			0x0C
#define REG_TS1_HI_BYTE			0x2C
#define REG_CC_HI_BYTE			0x32
#define REG_ADCGAIN1			0x50
#define REG_ADCOFFSET			0x51
#define REG_ADCGAIN2			0x59

// Macros
#define NTC_TEMP(res)			(1.0 / ((logf((res) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

// Variables
static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t bq_mutex;
static int m_offset = 0.0;
static int m_gain = 0.0;
static bool m_bal_cells[15] = {false};
static volatile bool m_bq_active = false;

static int m_cells = 13;

static esp_err_t i2c_tx_rx(uint8_t addr,
		const uint8_t* write_buffer, size_t write_size,
		uint8_t* read_buffer, size_t read_size) {

	if (!m_bq_active) {
		commands_printf_lisp("BQ76940 not active");
		return -2;
	}

	xSemaphoreTake(i2c_mutex, portMAX_DELAY);

	esp_err_t res;

	if (read_buffer != NULL) {
		res = i2c_master_write_read_device(0, addr, write_buffer, write_size, read_buffer, read_size, 500);
	} else {
		res = i2c_master_write_to_device(0, addr, write_buffer, write_size, 500);
	}
	xSemaphoreGive(i2c_mutex);

	return res;
}

static uint8_t crc8(uint8_t *ptr, uint8_t len, uint8_t key) {
	uint8_t i;
	uint8_t crc = 0;

	while (len-- != 0) {
		for(i = 0x80; i != 0; i /= 2) {
			if((crc & 0x80) != 0) {
				crc *= 2;
				crc ^= key;
			} else {
				crc *= 2;
			}

			if((*ptr & i) != 0) {
				crc ^= key;
			}
		}
		ptr++;
	}

	return(crc);
}

static bool bq_read_block(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
	uint8_t read_data[2 * len];
	esp_err_t res = i2c_tx_rx(addr, &reg, 1, read_data, 2 * len);
	uint8_t *read_data_ptr = read_data;

	if (res != ESP_OK) {
		commands_printf_lisp("I2C Error: %d", res);
		return false;
	}

	uint8_t crcbuf[2];
	uint8_t crc = 0;
	crcbuf[0] = (addr << 1) + 1;
	crcbuf[1] = *read_data_ptr;
	crc = crc8(crcbuf, 2, CRC_KEY);

	read_data_ptr++;
	if (crc != *read_data_ptr) {
		commands_printf_lisp("Bad CRC1");
		return false;
	} else {
		*buf = *(read_data_ptr - 1);
	}

	for (int i = 1; i < len; i++) {
		read_data_ptr++;
		crc = crc8(read_data_ptr, 1, CRC_KEY);
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

static bool bq_write_block(uint8_t addr, uint8_t start_addr, uint8_t *buf, uint8_t len) {
	uint8_t txbuf[2 * len + 2];
	uint8_t *bufptr = txbuf;
	*bufptr = addr << 1; bufptr++;
	*bufptr = start_addr; bufptr++;
	*bufptr = *buf; bufptr++;
	*bufptr = crc8(txbuf, 3, CRC_KEY);

	for(int i = 1; i < len; i++) {
        bufptr++;
        buf++;
        *bufptr = *buf;
		*(bufptr + 1) = crc8(bufptr, 1, CRC_KEY);
		bufptr++;
	}

	esp_err_t res = i2c_tx_rx(addr, txbuf + 1, 2 * len + 1, NULL, 0);

	return res == ESP_OK;
}

static int bq_read_reg(uint8_t reg) {
	uint8_t buf[1];
	bool res = bq_read_block(BQ_ADDR, reg, buf, 1);

	if (res) {
		return buf[0];
	} else {
		return -1;
	}
}

static bool bq_write_reg(uint8_t reg, uint8_t val) {
	uint8_t buf[1] = {val};
	return bq_write_block(BQ_ADDR, reg, buf, 1);
}

// Extensions
static lbm_value ext_bms_init(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	xSemaphoreTake(bq_mutex, portMAX_DELAY);

	gpio_config_t gpconf = {0};

	// Outputs

	gpconf.pin_bit_mask = BIT(PIN_OUT_EN) | BIT(PIN_CHG_EN) | BIT(PIN_PCHG_EN) |
			BIT(PIN_CAN_STB) | BIT(PIN_PSW_EN);
	gpconf.intr_type =  GPIO_FLOATING;
	gpconf.mode = GPIO_MODE_INPUT_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);

	gpio_set_level(PIN_OUT_EN, 0);
	gpio_set_level(PIN_CHG_EN, 0);
	gpio_set_level(PIN_PCHG_EN, 0);
	gpio_set_level(PIN_CAN_STB, 0);
	gpio_set_level(PIN_PSW_EN, 1);

	// Inputs

	gpconf.pin_bit_mask = BIT(PIN_ENABLE);
	gpconf.intr_type =  GPIO_FLOATING;
	gpconf.mode = GPIO_MODE_INPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);

	// Toggle boot high
	gpconf.pin_bit_mask = BIT(PIN_BOOT);
	gpconf.intr_type =  GPIO_FLOATING;
	gpconf.mode = GPIO_MODE_INPUT_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&gpconf);
	gpio_set_level(PIN_BOOT, 1);
	vTaskDelay(1);
	gpio_reset_pin(PIN_BOOT);

	vTaskDelay(10);

	// Restart i2c

	i2c_driver_delete(0);

	i2c_config_t conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = PIN_SDA,
			.scl_io_num = PIN_SCL,
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
			.master.clk_speed = 100000,
	};

	i2c_param_config(0, &conf);
	i2c_driver_install(0, conf.mode, 0, 0, 0);

	i2c_reset_tx_fifo(0);
	i2c_reset_rx_fifo(0);

	m_bq_active = true;

	// Should be written to 0x19 according to datasheet
	bool res = bq_write_reg(REG_CC_CFG, 0x19);

	// Enable ADC and choose external temperature sensors
	bq_write_reg(REG_SYS_CTRL1, 0x18);

	// Enable CC
	bq_write_reg(REG_SYS_CTRL2, 0x40);

	// Read factory-trimmed gain and offset
	int gain1 = bq_read_reg(REG_ADCGAIN1);
	int gain2 = bq_read_reg(REG_ADCGAIN2);
	m_offset = bq_read_reg(REG_ADCOFFSET);
	m_gain = 365 + ((gain1 & 0x0C) << 1) + ((gain2 & 0xE0) >> 5);

	xSemaphoreGive(bq_mutex);

	return res ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_hw_sleep(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	xSemaphoreTake(bq_mutex, portMAX_DELAY);

	// Disable all switches
	gpio_set_level(PIN_OUT_EN, 0);
	gpio_set_level(PIN_CHG_EN, 0);
	gpio_set_level(PIN_PCHG_EN, 0);
	gpio_set_level(PIN_PSW_EN, 0);

	// Put CAN-bus in standby mode
	gpio_set_level(PIN_CAN_STB, 1);

	// Stop balancing
	bq_write_reg(REG_CELLBAL1, 0);
	bq_write_reg(REG_CELLBAL2, 0);
	bq_write_reg(REG_CELLBAL3, 0);

	// Clear BQ76940 status
	bq_write_reg(REG_SYS_STAT, 0xFF);

	// Put BQ76940 in ship mode
	uint8_t ctrl1 = bq_read_reg(REG_SYS_CTRL1);

	ctrl1 &= ~0x03;
	bq_write_reg(REG_SYS_CTRL1, ctrl1);

	// SHUT_A=0, SHUT_B=1
	ctrl1 |= 0x01;
	bq_write_reg(REG_SYS_CTRL1, ctrl1);

	// SHUT_A=1, SHUT_B=0
	ctrl1 &= ~0x03;
	ctrl1 |= 0x02;
	bq_write_reg(REG_SYS_CTRL1, ctrl1);

	m_bq_active = false;

	xSemaphoreGive(bq_mutex);

	return ENC_SYM_TRUE;
}

static lbm_value ext_get_vcells(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	uint8_t buf[30];
	bool res = bq_read_block(BQ_ADDR, REG_VC1_HI_BYTE, buf, 30);

	if (res) {
		lbm_value vc_list = ENC_SYM_NIL;
		for (int i = 0;i < 15;i++) {

			// Skip cells that are not connected
			if (m_cells < 15 && i == 13) {
				i++;
			}

			if (m_cells < 14 && i == 8) {
				i++;
			}

			if (m_cells < 13 && i == 3) {
				i++;
			}

			int vc = (uint16_t)buf[2 * i] << 8 | (uint16_t)buf[2 * i + 1];
			vc = (vc * m_gain) / 1000;
			vc += m_offset;
			vc_list = lbm_cons(lbm_enc_float((float)vc / 1000.0), vc_list);
		}

		return lbm_list_destructive_reverse(vc_list);
	} else {
		return ENC_SYM_NIL;
	}
}

static lbm_value ext_get_temps(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	uint8_t buf[6];
	bool res = bq_read_block(BQ_ADDR, REG_TS1_HI_BYTE, buf, 6);

	if (res) {
		lbm_value ts_list = ENC_SYM_NIL;
		for (int i = 0;i < 3;i++) {
			int adc = (uint16_t)buf[2 * i] << 8 | (uint16_t)buf[2 * i + 1];
			float vts = ((float)adc * 382.0) * 1.0e-6;
			float rts = (10000.0 * vts) / (3.3 - vts);
			ts_list = lbm_cons(lbm_enc_float(NTC_TEMP(rts)), ts_list);
		}

		return lbm_list_destructive_reverse(ts_list);
	} else {
		return ENC_SYM_NIL;
	}
}

static lbm_value ext_get_current(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	uint8_t buf[2];
	bool res = bq_read_block(BQ_ADDR, REG_CC_HI_BYTE, buf, 2);

	if (res) {
		int16_t adc = (int16_t)(uint16_t)((uint16_t)buf[0] << 8 | (uint16_t)buf[1]);
		float current = -((float)adc * 8.44e-6) / HW_R_SHUNT;
		return lbm_enc_float(current);
	} else {
		return ENC_SYM_NIL;
	}
}

static lbm_value ext_read_reg(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	return lbm_enc_i(bq_read_reg(lbm_dec_as_u32(args[0])));
}

static lbm_value ext_write_reg(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);
	return bq_write_reg(lbm_dec_as_u32(args[0]), lbm_dec_as_u32(args[1])) ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_get_vout(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_float(HW_GET_VOUT());
}

static lbm_value ext_get_vchg(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_float(HW_GET_VCHG());
}

static lbm_value ext_get_btn(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_i(gpio_get_level(PIN_ENABLE) == 1 ? 0 : 1);
}

static lbm_value ext_set_btn_wakeup_state(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	switch (lbm_dec_as_i32(args[0])) {
	case 0:
		esp_deep_sleep_enable_gpio_wakeup(1 << PIN_ENABLE, ESP_GPIO_WAKEUP_GPIO_HIGH);
		break;

	case 1:
		esp_deep_sleep_enable_gpio_wakeup(1 << PIN_ENABLE, ESP_GPIO_WAKEUP_GPIO_LOW);
		break;

	default:
		gpio_deep_sleep_wakeup_disable(PIN_ENABLE);
		break;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_set_pchg(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	gpio_set_level(PIN_PCHG_EN, lbm_dec_as_i32(args[0]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_set_out(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	gpio_set_level(PIN_OUT_EN, lbm_dec_as_i32(args[0]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_set_chg(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	gpio_set_level(PIN_CHG_EN, lbm_dec_as_i32(args[0]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_set_bal(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int ch = lbm_dec_as_i32(args[0]);
	int state = lbm_dec_as_i32(args[1]);

	if (ch >= m_cells || !m_bq_active) {
		return ENC_SYM_NIL;
	} else {
		m_bal_cells[ch] = state;
		return ENC_SYM_TRUE;
	}
}

static lbm_value ext_get_bal(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int ch = lbm_dec_as_i32(args[0]);

	if (ch >= m_cells) {
		return ENC_SYM_NIL;
	} else {
		return lbm_enc_i(m_bal_cells[ch] ? 1 : 0);
	}
}

static lbm_value ext_set_cells(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int cells = lbm_dec_as_i32(args[0]);

	if (cells == 12 || cells == 13 || cells == 14 || cells == 15) {
		m_cells = cells;
		return ENC_SYM_TRUE;
	}

	lbm_set_error_reason("Invalid cell count");
	return ENC_SYM_TERROR;
}

typedef struct {
	lbm_uint balance_mode;
	lbm_uint max_bal_ch;
	lbm_uint dist_bal;
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
	lbm_uint i_measure_mode;
	lbm_uint sleep_timeout_reset_ms;
	lbm_uint min_charge_current;
	lbm_uint max_charge_current;
	lbm_uint soc_filter_const;
	lbm_uint t_bal_lim_start;
	lbm_uint t_bal_lim_end;
	lbm_uint t_charge_min;
	lbm_uint t_charge_mon_en;
} vesc_syms;

static vesc_syms syms_vesc = {0};

static bool get_add_symbol(char *name, lbm_uint* id) {
	if (!lbm_get_symbol_by_name(name, id)) {
		if (!lbm_add_symbol_const(name, id)) {
			return false;
		}
	}

	return true;
}

static bool compare_symbol(lbm_uint sym, lbm_uint *comp) {
	if (*comp == 0) {
		if (comp == &syms_vesc.balance_mode) {
			get_add_symbol("balance_mode", comp);
		} else if (comp == &syms_vesc.max_bal_ch) {
			get_add_symbol("max_bal_ch", comp);
		} else if (comp == &syms_vesc.dist_bal) {
			get_add_symbol("dist_bal", comp);
		} else if (comp == &syms_vesc.vc_balance_start) {
			get_add_symbol("vc_balance_start", comp);
		} else if (comp == &syms_vesc.vc_balance_end) {
			get_add_symbol("vc_balance_end", comp);
		} else if (comp == &syms_vesc.vc_charge_start) {
			get_add_symbol("vc_charge_start", comp);
		} else if (comp == &syms_vesc.vc_charge_end) {
			get_add_symbol("vc_charge_end", comp);
		} else if (comp == &syms_vesc.vc_charge_min) {
			get_add_symbol("vc_charge_min", comp);
		} else if (comp == &syms_vesc.vc_balance_min) {
			get_add_symbol("vc_balance_min", comp);
		} else if (comp == &syms_vesc.balance_max_current) {
			get_add_symbol("balance_max_current", comp);
		} else if (comp == &syms_vesc.min_current_ah_wh_cnt) {
			get_add_symbol("min_current_ah_wh_cnt", comp);
		} else if (comp == &syms_vesc.min_current_sleep) {
			get_add_symbol("min_current_sleep", comp);
		} else if (comp == &syms_vesc.v_charge_detect) {
			get_add_symbol("v_charge_detect", comp);
		} else if (comp == &syms_vesc.t_charge_max) {
			get_add_symbol("t_charge_max", comp);
		} else if (comp == &syms_vesc.i_measure_mode) {
			get_add_symbol("i_measure_mode", comp);
		} else if (comp == &syms_vesc.sleep_timeout_reset_ms) {
			get_add_symbol("sleep_timeout_reset_ms", comp);
		} else if (comp == &syms_vesc.min_charge_current) {
			get_add_symbol("min_charge_current", comp);
		} else if (comp == &syms_vesc.max_charge_current) {
			get_add_symbol("max_charge_current", comp);
		} else if (comp == &syms_vesc.soc_filter_const) {
			get_add_symbol("soc_filter_const", comp);
		} else if (comp == &syms_vesc.t_bal_lim_start) {
			get_add_symbol("t_bal_lim_start", comp);
		} else if (comp == &syms_vesc.t_bal_lim_end) {
			get_add_symbol("t_bal_lim_end", comp);
		} else if (comp == &syms_vesc.t_charge_min) {
			get_add_symbol("t_charge_min", comp);
		} else if (comp == &syms_vesc.t_charge_mon_en) {
			get_add_symbol("t_charge_mon_en", comp);
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
			lbm_set_error_reason((char*)lbm_error_str_no_number);
			return ENC_SYM_EERROR;
		}
	}

	if (argn != 1 && argn != 2) {
		return res;
	}

	if (lbm_type_of(args[0]) != LBM_TYPE_SYMBOL) {
		return res;
	}

	lbm_uint name = lbm_dec_sym(args[0]);
	main_config_t *cfg = (main_config_t*)&backup.config;

	if (compare_symbol(name, &syms_vesc.balance_mode)) {
		int tmp = cfg->balance_mode;
		res = get_or_set_i(set, &tmp, &set_arg);
		cfg->balance_mode = tmp;
	} else if (compare_symbol(name, &syms_vesc.max_bal_ch)) {
		res = get_or_set_i(set, &cfg->max_bal_ch, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.dist_bal)) {
		res = get_or_set_bool(set, &cfg->dist_bal, &set_arg);
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
	} else if (compare_symbol(name, &syms_vesc.i_measure_mode)) {
		int tmp = cfg->i_measure_mode;
		res = get_or_set_i(set, &tmp, &set_arg);
		cfg->i_measure_mode = tmp;
	} else if (compare_symbol(name, &syms_vesc.sleep_timeout_reset_ms)) {
		res = get_or_set_i(set, &cfg->sleep_timeout_reset_ms, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.min_charge_current)) {
		res = get_or_set_float(set, &cfg->min_charge_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.max_charge_current)) {
		res = get_or_set_float(set, &cfg->max_charge_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.soc_filter_const)) {
		res = get_or_set_float(set, &cfg->soc_filter_const, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_bal_lim_start)) {
		res = get_or_set_float(set, &cfg->t_bal_lim_start, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_bal_lim_end)) {
		res = get_or_set_float(set, &cfg->t_bal_lim_end, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_min)) {
		res = get_or_set_float(set, &cfg->t_charge_min, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_mon_en)) {
		res = get_or_set_bool(set, &cfg->t_charge_mon_en, &set_arg);
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
	(void)args; (void)argn;
	main_store_backup_data();
	return ENC_SYM_TRUE;
}

static void load_extensions(void) {
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

	// Read and write balance IC registers
	lbm_add_extension("bms-read-reg", ext_read_reg);
	lbm_add_extension("bms-write-reg", ext_write_reg);

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

	lbm_add_extension("bms-set-cells", ext_set_cells);

	// Configuration
	lbm_add_extension("bms-get-param", ext_bms_get_param);
	lbm_add_extension("bms-set-param", ext_bms_set_param);
	lbm_add_extension("bms-store-cfg", ext_bms_store_cfg);
}

static void bal_task(void *arg) {
	// Balancing adjacent cells is not allowed, so split cells into two groups and alter
	// between them every second.

	bool is_grp2 = false;

	for (;;) {
		bool bal_now[m_cells];
		memset(bal_now, 0, sizeof(bal_now));

		for (int i = (is_grp2 ? 0 : 1);i < m_cells;i += 2) {
			bal_now[i] = m_bal_cells[i];
		}

		uint8_t cb1 = 0;
		uint8_t cb2 = 0;
		uint8_t cb3 = 0;

		int ch = 0;

		cb1 |= (bal_now[ch++] ? 1 : 0) << 0; // CB1
		cb1 |= (bal_now[ch++] ? 1 : 0) << 1; // CB2
		cb1 |= (bal_now[ch++] ? 1 : 0) << 2; // CB3
		if (m_cells >= 13) {
			cb1 |= (bal_now[ch++] ? 1 : 0) << 3; // CB4
		}
		cb1 |= (bal_now[ch++] ? 1 : 0) << 4; // CB5

		cb2 |= (bal_now[ch++] ? 1 : 0) << 0; // CB6
		cb2 |= (bal_now[ch++] ? 1 : 0) << 1; // CB7
		cb2 |= (bal_now[ch++] ? 1 : 0) << 2; // CB8
		if (m_cells >= 14) {
			cb2 |= (bal_now[ch++] ? 1 : 0) << 3; // CB9
		}
		cb2 |= (bal_now[ch++] ? 1 : 0) << 4; // CB10

		cb3 |= (bal_now[ch++] ? 1 : 0) << 0; // CB11
		cb3 |= (bal_now[ch++] ? 1 : 0) << 1; // CB12
		cb3 |= (bal_now[ch++] ? 1 : 0) << 2; // CB13
		if (m_cells >= 15) {
			cb3 |= (bal_now[ch++] ? 1 : 0) << 3; // CB14
		}
		cb3 |= (bal_now[ch++] ? 1 : 0) << 4; // CB15

		xSemaphoreTake(bq_mutex, portMAX_DELAY);

		if (m_bq_active) {
			bq_write_reg(REG_CELLBAL1, cb1);
			bq_write_reg(REG_CELLBAL2, cb2);
			bq_write_reg(REG_CELLBAL3, cb3);
		} else {
			memset(m_bal_cells, 0, sizeof(m_bal_cells));
		}

		xSemaphoreGive(bq_mutex);

		vTaskDelay(1000 / portTICK_PERIOD_MS);
		is_grp2 = !is_grp2;
	}
}

void hw_init(void) {
	i2c_mutex = xSemaphoreCreateMutex();
	bq_mutex = xSemaphoreCreateMutex();

	i2c_config_t conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = PIN_SDA,
			.scl_io_num = PIN_SCL,
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
			.master.clk_speed = 100000,
	};

	i2c_param_config(0, &conf);
	i2c_driver_install(0, conf.mode, 0, 0, 0);

	lispif_add_ext_load_callback(load_extensions);

	xTaskCreatePinnedToCore(bal_task, "balance", 1024, NULL, 6, NULL, tskNO_AFFINITY);
}
