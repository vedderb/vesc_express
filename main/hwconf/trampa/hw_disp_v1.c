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

#include "hw_disp_v1.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "driver/gpio.h"
#include "lispif_disp_extensions.h"
#include "disp_st7789.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_sleep.h"

#include "lispif.h"
#include "lispbm.h"
#include "terminal.h"
#include "commands.h"
#include "utils.h"

static float v_ext = 0.0;
static float v_btn = 0.0;

#if DISP_HW_VERSION == DISP_V1_3
static int pin_3v3 = 4;
static int pin_btn = 2;
#elif DISP_HW_VERSION == DISP_V1_2
static int pin_bl = 5;
static int pin_3v3 = 21;
static int pin_psw = 2;
static int pin_btn = 3;
#elif DISP_HW_VERSION == DISP_V0
static int pin_bl = 3;
static int pin_3v3 = 21;
static int pin_psw = -1;
static int pin_btn = 2;
#else
#error "Incompatible DISP_HW_VERSION"
#endif



static void hw_task(void *arg) {
	for(;;) {
		UTILS_LP_FAST(v_ext, adc_get_voltage(HW_ADC_CH1), 0.1);
#if DISP_HW_VERSION == DISP_V0
		UTILS_LP_FAST(v_btn, adc_get_voltage(HW_ADC_CH3), 0.1);
#else
		UTILS_LP_FAST(v_btn, adc_get_voltage(HW_ADC_CH0), 0.1);
#endif
		vTaskDelay(1);
	}
}

static lbm_value ext_v_ext(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_float(v_ext);
}

static lbm_value ext_v_btn(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_float(v_btn);
}

#if DISP_HW_VERSION == DISP_V1_3
#define GPIO_EXP_INPUT_REG  0x00
#define GPIO_EXP_OUTPUT_REG 0x01
#define GPIO_EXP_CONFIG_REG 0x03

static SemaphoreHandle_t 	i2c_mutex;

static void i2c_init(void) {
	i2c_mutex = xSemaphoreCreateMutex();

	i2c_config_t conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = I2C_SDA,
			.scl_io_num = I2C_SCL,
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
			.master.clk_speed = 100000,
	};
	i2c_param_config(0, &conf);
	i2c_driver_install(0, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_tx_rx(uint8_t addr,
		const uint8_t* write_buffer, size_t write_size,
		uint8_t* read_buffer, size_t read_size) {

	xSemaphoreTake(i2c_mutex, portMAX_DELAY);

	esp_err_t res;
	if (read_size > 0 && read_buffer != NULL) {
		if (write_size > 0 && write_buffer != NULL) {
			res = i2c_master_write_read_device(0, addr, write_buffer, write_size, read_buffer, read_size, 2000);
		} else {
			res = i2c_master_read_from_device(0, addr, read_buffer, read_size, 2000);
		}
	} else {
		res = i2c_master_write_to_device(0, addr, write_buffer, write_size, 2000);
	}

	xSemaphoreGive(i2c_mutex);

	return res;
}

static int i2c_read_reg(uint8_t addr, uint8_t reg) {
	uint8_t tx_buf[1] = {reg};
	uint8_t rx_buf[1];
	esp_err_t e = i2c_tx_rx(addr, tx_buf, 1, rx_buf, 1);
	if (e == ESP_OK) {
		return rx_buf[0];
	} else {
		return -1;
	}
}

static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
	uint8_t tx_buf[2] = {reg, val};
	return i2c_tx_rx(addr, tx_buf, 2, 0, 0);
}

void init_gpio_expander(void) {
	i2c_write_reg(I2C_ADDR_GPIO_EXP, GPIO_EXP_CONFIG_REG, 0x07); // Ports 0-2 Input
	i2c_write_reg(I2C_ADDR_GPIO_EXP, GPIO_EXP_OUTPUT_REG, 0x00); // All Output Disable
}

static lbm_value ext_set_io(lbm_value *args, lbm_uint argn) {
	if (argn == 2 && lbm_is_number(args[0]) && lbm_is_number(args[1])) {
		lbm_uint i = lbm_dec_as_u32(args[0]);
		lbm_uint v = lbm_dec_as_u32(args[1]);
		uint8_t reg = i2c_read_reg(I2C_ADDR_GPIO_EXP, GPIO_EXP_OUTPUT_REG);
		if (i <= 7) {
			if (v == 0) {
				reg &= ~(1 << i);
			} else {
				reg |= (1 << i);
			}
		}
		i2c_write_reg(I2C_ADDR_GPIO_EXP, GPIO_EXP_OUTPUT_REG, (uint8_t)reg);
		return ENC_SYM_TRUE;
	}
	return ENC_SYM_TERROR;
}

static lbm_value ext_read_button(lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_TERROR;
	if (argn == 1 && lbm_is_number(args[0])) {
		int32_t button = lbm_dec_as_i32(args[0]);
		button = button & 0x7;

		//commands_printf_lisp("button = %d\n", button);

		res = ENC_SYM_NIL;

		if (button == 0) {
			if (v_btn > 2.0) {
				res = ENC_SYM_TRUE;
			}
		} else {
			uint8_t io = i2c_read_reg(I2C_ADDR_GPIO_EXP, GPIO_EXP_INPUT_REG);
			//commands_printf_lisp("io = %x\n", io);
			if (io & (1 << (button - 1))) {
				res = ENC_SYM_TRUE;
			}
		}
	}
	return res;
}
#endif

static lbm_value ext_hw_init(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

#if DISP_HW_VERSION != DISP_V1_3
	gpio_reset_pin(pin_bl);
	gpio_set_direction(pin_bl, GPIO_MODE_OUTPUT);
	gpio_set_level(pin_bl, 0);

	if (pin_psw >= 0) {
		gpio_reset_pin(pin_psw);
		gpio_set_direction(pin_psw, GPIO_MODE_OUTPUT);
		gpio_set_level(pin_psw, 1);
	}
#endif

	gpio_reset_pin(pin_3v3);
	gpio_set_direction(pin_3v3, GPIO_MODE_OUTPUT);
	gpio_set_level(pin_3v3, 1);

	disp_st7789_init(7, 6, 10, 20, 8, 40);

	lispif_disp_set_callbacks(
			disp_st7789_render_image,
			disp_st7789_clear,
			disp_st7789_reset
	);

	disp_st7789_reset();
	disp_st7789_clear(0x00);

	// Orientation
	uint8_t arg = 0xA0;
	disp_st7789_command(0x36, &arg, 1);

#if DISP_HW_VERSION == DISP_V1_3
	i2c_write_reg(I2C_ADDR_GPIO_EXP, GPIO_EXP_OUTPUT_REG, 0x08); // Port 3 (DISP_LED) Output Enable
#else
	gpio_set_level(pin_bl, 1);
#endif

	return ENC_SYM_TRUE;
}

static lbm_value ext_hw_sleep(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	esp_bluedroid_disable();
	esp_bt_controller_disable();
	esp_wifi_stop();

#if DISP_HW_VERSION == DISP_V1_3
	i2c_write_reg(I2C_ADDR_GPIO_EXP, GPIO_EXP_OUTPUT_REG, 0x10); // Port 4 (PWR SW) Output Enable
	while (v_btn > 2.0) {
		vTaskDelay(5);
	}

	gpio_set_direction(pin_btn, GPIO_MODE_INPUT);
	esp_deep_sleep_enable_gpio_wakeup(1 << pin_btn, ESP_GPIO_WAKEUP_GPIO_HIGH);
#else
	gpio_set_level(pin_bl, 0);
	while (v_btn < 2.0) {
		vTaskDelay(5);
	}

	gpio_set_direction(pin_btn, GPIO_MODE_INPUT);
	esp_deep_sleep_enable_gpio_wakeup(1 << pin_btn, ESP_GPIO_WAKEUP_GPIO_LOW);
#endif

	esp_deep_sleep_start();

	return ENC_SYM_TRUE;
}

static void load_extensions(void) {
	lbm_add_extension("v-ext", ext_v_ext);
	lbm_add_extension("v-btn", ext_v_btn);
	lbm_add_extension("hw-init", ext_hw_init);
	lbm_add_extension("hw-sleep", ext_hw_sleep);
#if DISP_HW_VERSION == DISP_V1_3
	lbm_add_extension("read-button", ext_read_button);
	lbm_add_extension("set-io", ext_set_io);
#endif
}

void hw_init(void) {

#if DISP_HW_VERSION == DISP_V1_3
	i2c_init();
	init_gpio_expander();
#endif

	xTaskCreatePinnedToCore(hw_task, "hw disp", 1024, NULL, 6, NULL, tskNO_AFFINITY);
	lispif_add_ext_load_callback(load_extensions);
}
