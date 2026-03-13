/*
	Copyright 2025

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

#include "lispif_touch_extensions.h"

#include "lispif_events.h"
#include "lbm_vesc_utils.h"
#include "eval_cps.h"
#include "extensions.h"
#include "heap.h"
#include "lbm_flat_value.h"
#include "utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_lcd_touch_cst9217.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_lcd_touch_xpt2046.h"

#define TOUCH_I2C_PORT I2C_NUM_0
#define TOUCH_I2C_DEFAULT_FREQ 400000
#define TOUCH_EVENT_TASK_STACK 2048

static char *msg_invalid_gpio = "Invalid GPIO";
static char *msg_invalid_size = "Invalid touch size";
static char *msg_invalid_i2c_speed = "Invalid I2C speed";
static char *msg_invalid_spi_host = "Invalid SPI host";
static char *msg_touch_not_loaded = "Touch not loaded";
static char *msg_touch_runtime = "Touch runtime init failed";

static SemaphoreHandle_t touch_mutex = 0;
static TaskHandle_t touch_event_task_handle = 0;
static lispif_touch_driver_t touch_driver = {0};
static esp_lcd_touch_handle_t touch_handle = NULL;
static esp_lcd_panel_io_handle_t touch_io_handle = NULL;
static i2c_port_t touch_i2c_port = TOUCH_I2C_PORT;
static bool touch_owns_i2c_driver = false;
static spi_host_device_t touch_spi_host = SPI2_HOST;
static bool touch_owns_spi_bus = false;
static lbm_uint touch_driver_symbol = 0;

static lbm_uint sym_cst816s = 0;
static lbm_uint sym_gt911 = 0;
static lbm_uint sym_cst9217 = 0;
static lbm_uint sym_xpt2046 = 0;

typedef esp_err_t (*touch_i2c_create_fn_t)(esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *cfg, esp_lcd_touch_handle_t *tp);

static void touch_event_task(void *arg);
static void touch_esp_lcd_interrupt_cb(esp_lcd_touch_handle_t tp);
static esp_err_t touch_esp_lcd_deinit(void);
static esp_err_t touch_esp_lcd_read_data(void);
static esp_err_t touch_esp_lcd_get_data(lispif_touch_point_data_t *data, uint8_t *point_cnt, uint8_t max_point_cnt);
static esp_err_t touch_init_i2c_bus(int sda, int scl, uint32_t freq);
static esp_err_t touch_init_i2c_esp_lcd(int sda, int scl, int rst, int int_pin, uint16_t width, uint16_t height, uint32_t freq, esp_lcd_panel_io_i2c_config_t io_conf, void *driver_data, touch_i2c_create_fn_t create_fn, lispif_touch_driver_t *driver);
static esp_err_t touch_init_cst816s_esp_lcd(int sda, int scl, int rst, int int_pin, uint16_t width, uint16_t height, uint32_t freq, lispif_touch_driver_t *driver);
static esp_err_t touch_init_gt911_esp_lcd(int sda, int scl, int rst, int int_pin, uint16_t width, uint16_t height, uint32_t freq, lispif_touch_driver_t *driver);
static esp_err_t touch_init_cst9217_esp_lcd(int sda, int scl, int rst, int int_pin, uint16_t width, uint16_t height, uint32_t freq, lispif_touch_driver_t *driver);
static esp_err_t touch_init_xpt2046_esp_lcd(int host, int mosi, int miso, int sclk, int cs, int int_pin, uint16_t width, uint16_t height, uint32_t freq, lispif_touch_driver_t *driver);
static bool touch_lock_loaded_or_error(void);

static bool start_flatten_with_gc(lbm_flat_value_t *v, size_t buffer_size) {
	if (lbm_start_flatten(v, buffer_size)) {
		return true;
	}

	int timeout = 3;
	uint32_t gc_last = lbm_heap_state.gc_num;
	lbm_request_gc();

	while (lbm_heap_state.gc_num <= gc_last && timeout > 0) {
		vTaskDelay(1);
		timeout--;
	}

	return lbm_start_flatten(v, buffer_size);
}

static bool touch_gpio_is_valid_or_nc(int pin) {
	return pin == -1 || utils_gpio_is_valid(pin);
}

static bool touch_runtime_init(void) {
	if (!touch_mutex) {
		touch_mutex = xSemaphoreCreateMutex();
		if (!touch_mutex) {
			return false;
		}
	}

	if (!touch_event_task_handle) {
		if (xTaskCreatePinnedToCore(
				touch_event_task,
				"touch_evt",
				TOUCH_EVENT_TASK_STACK,
				NULL,
				6,
				&touch_event_task_handle,
				tskNO_AFFINITY) != pdPASS) {
			return false;
		}
	}

	return true;
}

static bool touch_driver_loaded(void) {
	return touch_driver.read_data != 0 && touch_driver.get_data != 0;
}

static void IRAM_ATTR touch_esp_lcd_interrupt_cb(esp_lcd_touch_handle_t tp) {
	(void)tp;
	lispif_touch_irq_from_isr();
}

static esp_err_t touch_esp_lcd_deinit(void) {
	esp_err_t first_err = ESP_OK;

	if (touch_handle) {
		esp_err_t res = touch_handle->del(touch_handle);
		if (res != ESP_OK && first_err == ESP_OK) {
			first_err = res;
		}
		touch_handle = NULL;
	}

	if (touch_io_handle) {
		esp_err_t res = esp_lcd_panel_io_del(touch_io_handle);
		if (res != ESP_OK && first_err == ESP_OK) {
			first_err = res;
		}
		touch_io_handle = NULL;
	}

	if (touch_owns_i2c_driver) {
		esp_err_t res = i2c_driver_delete(touch_i2c_port);
		if (res != ESP_OK && first_err == ESP_OK) {
			first_err = res;
		}
	}

	touch_owns_i2c_driver = false;

	if (touch_owns_spi_bus) {
		esp_err_t res = spi_bus_free(touch_spi_host);
		if (res != ESP_OK && first_err == ESP_OK) {
			first_err = res;
		}
	}

	touch_owns_spi_bus = false;
	return first_err;
}

static esp_err_t touch_esp_lcd_read_data(void) {
	if (!touch_handle) {
		return ESP_ERR_INVALID_STATE;
	}

	return esp_lcd_touch_read_data(touch_handle);
}

static esp_err_t touch_esp_lcd_get_data(lispif_touch_point_data_t *data, uint8_t *point_cnt, uint8_t max_point_cnt) {
	if (!touch_handle) {
		return ESP_ERR_INVALID_STATE;
	}

	if (!data || !point_cnt || max_point_cnt == 0) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t req_cnt = max_point_cnt;
	if (req_cnt > CONFIG_ESP_LCD_TOUCH_MAX_POINTS) {
		req_cnt = CONFIG_ESP_LCD_TOUCH_MAX_POINTS;
	}

	esp_lcd_touch_point_data_t points[CONFIG_ESP_LCD_TOUCH_MAX_POINTS];
	uint8_t cnt = 0;
	esp_err_t res = esp_lcd_touch_get_data(touch_handle, points, &cnt, req_cnt);
	if (res != ESP_OK) {
		return res;
	}

	for (uint8_t i = 0;i < cnt;i++) {
		data[i].track_id = points[i].track_id;
		data[i].x = points[i].x;
		data[i].y = points[i].y;
		data[i].strength = points[i].strength;
	}

	*point_cnt = cnt;
	return ESP_OK;
}

static esp_err_t touch_init_i2c_bus(int sda, int scl, uint32_t freq) {
	i2c_config_t i2c_conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = sda,
			.scl_io_num = scl,
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
			.master.clk_speed = freq,
	};

	bool reuse_existing_i2c = false;
	esp_err_t cfg_res = i2c_param_config(TOUCH_I2C_PORT, &i2c_conf);
	if (cfg_res == ESP_ERR_INVALID_ARG) {
		i2c_conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
		i2c_conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
		cfg_res = i2c_param_config(TOUCH_I2C_PORT, &i2c_conf);
	}
	if (cfg_res != ESP_OK) {
		if (cfg_res != ESP_FAIL && cfg_res != ESP_ERR_INVALID_STATE) {
			return cfg_res;
		}
		reuse_existing_i2c = true;
	}

	esp_err_t res = i2c_driver_install(TOUCH_I2C_PORT, i2c_conf.mode, 0, 0, 0);
	if (res == ESP_ERR_INVALID_STATE) {
		reuse_existing_i2c = true;
	} else if (res != ESP_OK) {
		return res;
	}

	touch_owns_i2c_driver = !reuse_existing_i2c;
	touch_i2c_port = TOUCH_I2C_PORT;
	return ESP_OK;
}

static esp_err_t touch_init_i2c_esp_lcd(int sda, int scl, int rst, int int_pin, uint16_t width, uint16_t height, uint32_t freq, esp_lcd_panel_io_i2c_config_t io_conf, void *driver_data, touch_i2c_create_fn_t create_fn, lispif_touch_driver_t *driver) {
	if (!driver || !create_fn) {
		return ESP_ERR_INVALID_ARG;
	}

	if (touch_handle || touch_io_handle) {
		esp_err_t deinit_res = touch_esp_lcd_deinit();
		if (deinit_res != ESP_OK) {
			return deinit_res;
		}
	}

	esp_err_t res = touch_init_i2c_bus(sda, scl, freq);
	if (res != ESP_OK) {
		return res;
	}

	io_conf.scl_speed_hz = 0;
	res = esp_lcd_new_panel_io_i2c(TOUCH_I2C_PORT, &io_conf, &touch_io_handle);
	if (res != ESP_OK) {
		touch_esp_lcd_deinit();
		return res;
	}

	esp_lcd_touch_config_t tp_cfg = {
			.x_max = width,
			.y_max = height,
			.rst_gpio_num = rst >= 0 ? (gpio_num_t)rst : GPIO_NUM_NC,
			.int_gpio_num = int_pin >= 0 ? (gpio_num_t)int_pin : GPIO_NUM_NC,
			.levels = {
					.reset = 0,
					.interrupt = 0,
			},
			.flags = {
					.swap_xy = 0,
					.mirror_x = 0,
					.mirror_y = 0,
			},
			.interrupt_callback = touch_esp_lcd_interrupt_cb,
			.driver_data = driver_data,
	};

	res = create_fn(touch_io_handle, &tp_cfg, &touch_handle);
	if (res != ESP_OK) {
		touch_esp_lcd_deinit();
		return res;
	}

	driver->deinit = touch_esp_lcd_deinit;
	driver->read_data = touch_esp_lcd_read_data;
	driver->get_data = touch_esp_lcd_get_data;

	return ESP_OK;
}

static esp_err_t touch_init_cst816s_esp_lcd(int sda, int scl, int rst, int int_pin, uint16_t width, uint16_t height, uint32_t freq, lispif_touch_driver_t *driver) {
	esp_lcd_panel_io_i2c_config_t io_conf = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
	return touch_init_i2c_esp_lcd(sda, scl, rst, int_pin, width, height, freq, io_conf, NULL, esp_lcd_touch_new_i2c_cst816s, driver);
}

static esp_err_t touch_init_gt911_esp_lcd(int sda, int scl, int rst, int int_pin, uint16_t width, uint16_t height, uint32_t freq, lispif_touch_driver_t *driver) {
	esp_lcd_panel_io_i2c_config_t io_conf = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
	esp_lcd_touch_io_gt911_config_t gt911_cfg = {
			.dev_addr = io_conf.dev_addr,
	};

	return touch_init_i2c_esp_lcd(sda, scl, rst, int_pin, width, height, freq, io_conf, &gt911_cfg, esp_lcd_touch_new_i2c_gt911, driver);
}

static esp_err_t touch_init_cst9217_esp_lcd(int sda, int scl, int rst, int int_pin, uint16_t width, uint16_t height, uint32_t freq, lispif_touch_driver_t *driver) {
	esp_lcd_panel_io_i2c_config_t io_conf = ESP_LCD_TOUCH_IO_I2C_CST9217_CONFIG();
	return touch_init_i2c_esp_lcd(sda, scl, rst, int_pin, width, height, freq, io_conf, NULL, esp_lcd_touch_new_i2c_cst9217, driver);
}

static esp_err_t touch_init_xpt2046_esp_lcd(int host, int mosi, int miso, int sclk, int cs, int int_pin, uint16_t width, uint16_t height, uint32_t freq, lispif_touch_driver_t *driver) {
	if (!driver) {
		return ESP_ERR_INVALID_ARG;
	}

	if (touch_handle || touch_io_handle) {
		esp_err_t deinit_res = touch_esp_lcd_deinit();
		if (deinit_res != ESP_OK) {
			return deinit_res;
		}
	}

	spi_host_device_t spi_host = (spi_host_device_t)host;
	spi_bus_config_t bus_cfg = {
			.mosi_io_num = mosi,
			.miso_io_num = miso,
			.sclk_io_num = sclk,
			.quadwp_io_num = -1,
			.quadhd_io_num = -1,
			.max_transfer_sz = 0,
	};

	bool reuse_existing_spi = false;
	esp_err_t res = spi_bus_initialize(spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
	if (res == ESP_ERR_INVALID_STATE) {
		reuse_existing_spi = true;
	} else if (res != ESP_OK) {
		return res;
	}

	touch_spi_host = spi_host;
	touch_owns_spi_bus = !reuse_existing_spi;

	esp_lcd_panel_io_spi_config_t io_conf = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(cs);
	io_conf.pclk_hz = freq;
	res = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)spi_host, &io_conf, &touch_io_handle);
	if (res != ESP_OK) {
		touch_esp_lcd_deinit();
		return res;
	}

	esp_lcd_touch_config_t tp_cfg = {
			.x_max = width,
			.y_max = height,
			.rst_gpio_num = GPIO_NUM_NC,
			.int_gpio_num = int_pin >= 0 ? (gpio_num_t)int_pin : GPIO_NUM_NC,
			.levels = {
					.reset = 0,
					.interrupt = 0,
			},
			.flags = {
					.swap_xy = 0,
					.mirror_x = 0,
					.mirror_y = 0,
			},
			.interrupt_callback = touch_esp_lcd_interrupt_cb,
	};

	res = esp_lcd_touch_new_spi_xpt2046(touch_io_handle, &tp_cfg, &touch_handle);
	if (res != ESP_OK) {
		touch_esp_lcd_deinit();
		return res;
	}

	driver->deinit = touch_esp_lcd_deinit;
	driver->read_data = touch_esp_lcd_read_data;
	driver->get_data = touch_esp_lcd_get_data;

	return ESP_OK;
}

static bool touch_lock_loaded_or_error(void) {
	if (!touch_mutex) {
		lbm_set_error_reason(msg_touch_not_loaded);
		return false;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	if (!touch_driver_loaded()) {
		xSemaphoreGive(touch_mutex);
		lbm_set_error_reason(msg_touch_not_loaded);
		return false;
	}

	return true;
}

static void touch_delete_locked(void) {
	if (touch_driver.deinit) {
		touch_driver.deinit();
	}

	memset(&touch_driver, 0, sizeof(touch_driver));
	touch_driver_symbol = 0;
}

static lbm_value touch_get_point_locked(void) {
	lispif_touch_point_data_t point;
	uint8_t point_cnt = 0;
	esp_err_t res = touch_driver.get_data(&point, &point_cnt, 1);
	if (res != ESP_OK) {
		lbm_set_esp_error_reason(res);
		return ENC_SYM_EERROR;
	}

	if (point_cnt == 0) {
		return ENC_SYM_NIL;
	}

	return make_list(
			4,
			lbm_enc_u(point.x),
			lbm_enc_u(point.y),
			lbm_enc_u(point.strength),
			lbm_enc_u(point.track_id));
}

static void touch_emit_event(bool pressed, const lispif_touch_point_data_t *point) {
	lbm_flat_value_t flat;
	if (!start_flatten_with_gc(&flat, 96)) {
		return;
	}

	f_cons(&flat);
	f_sym(&flat, sym_event_touch_int);

	f_cons(&flat);
	f_sym(&flat, touch_driver_symbol);

	f_cons(&flat);
	f_sym(&flat, pressed ? SYM_TRUE : SYM_NIL);

	f_cons(&flat);
	f_u(&flat, pressed ? point->x : 0);

	f_cons(&flat);
	f_u(&flat, pressed ? point->y : 0);

	f_cons(&flat);
	f_u(&flat, pressed ? point->strength : 0);

	f_cons(&flat);
	f_u(&flat, pressed ? point->track_id : 0);

	f_sym(&flat, SYM_NIL);
	lbm_finish_flatten(&flat);

	if (!lbm_event(&flat)) {
		lbm_free(flat.buf);
	}
}

void lispif_touch_irq_from_isr(void) {
	if (!event_touch_int_en || !touch_event_task_handle) {
		return;
	}

	BaseType_t wake = pdFALSE;
	vTaskNotifyGiveFromISR(touch_event_task_handle, &wake);
	portYIELD_FROM_ISR(wake);
}

void lispif_touch_shutdown(void) {
	if (!touch_mutex) {
		return;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	touch_delete_locked();
	xSemaphoreGive(touch_mutex);
}

static void touch_event_task(void *arg) {
	(void)arg;

	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if (!event_touch_int_en || !touch_mutex) {
			continue;
		}

		lispif_touch_point_data_t point = {0};
		bool pressed = false;

		xSemaphoreTake(touch_mutex, portMAX_DELAY);
		if (touch_driver_loaded()) {
			esp_err_t res = touch_driver.read_data();
			if (res == ESP_OK) {
				uint8_t point_cnt = 0;
				if (touch_driver.get_data(&point, &point_cnt, 1) == ESP_OK) {
					pressed = point_cnt > 0;
				}
			}
		}
		xSemaphoreGive(touch_mutex);

		if (event_touch_int_en) {
			touch_emit_event(pressed, &point);
		}
	}
}

static bool touch_validate_i2c_load_args(int pin_sda, int pin_scl, int pin_rst, int pin_int, int width, int height, int freq) {
	if (!utils_gpio_is_valid(pin_sda) ||
			!utils_gpio_is_valid(pin_scl) ||
			!touch_gpio_is_valid_or_nc(pin_rst) ||
			!touch_gpio_is_valid_or_nc(pin_int)) {
		lbm_set_error_reason(msg_invalid_gpio);
		return false;
	}

	if (width <= 0 || height <= 0) {
		lbm_set_error_reason(msg_invalid_size);
		return false;
	}

	if (freq <= 0) {
		lbm_set_error_reason(msg_invalid_i2c_speed);
		return false;
	}

	return true;
}

static lbm_value touch_load_driver(lispif_touch_driver_t driver, lbm_uint driver_symbol, int width, int height) {
	(void)width;
	(void)height;

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	touch_driver = driver;
	touch_driver_symbol = driver_symbol;
	xSemaphoreGive(touch_mutex);

	return ENC_SYM_TRUE;
}

static lbm_value ext_touch_load_cst816s(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_RANGE(6, 7);

	int pin_sda = lbm_dec_as_i32(args[0]);
	int pin_scl = lbm_dec_as_i32(args[1]);
	int pin_rst = lbm_dec_as_i32(args[2]);
	int pin_int = lbm_dec_as_i32(args[3]);
	int width = lbm_dec_as_i32(args[4]);
	int height = lbm_dec_as_i32(args[5]);
	int freq = TOUCH_I2C_DEFAULT_FREQ;

	if (argn == 7) {
		freq = lbm_dec_as_i32(args[6]);
	}

	if (!touch_validate_i2c_load_args(pin_sda, pin_scl, pin_rst, pin_int, width, height, freq)) {
		return ENC_SYM_EERROR;
	}

	if (!touch_runtime_init()) {
		lbm_set_error_reason(msg_touch_runtime);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	touch_delete_locked();
	xSemaphoreGive(touch_mutex);
	lispif_touch_driver_t driver = {0};
	esp_err_t res = touch_init_cst816s_esp_lcd(
			pin_sda,
			pin_scl,
			pin_rst,
			pin_int,
			(uint16_t)width,
			(uint16_t)height,
			(uint32_t)freq,
			&driver);
	if (res != ESP_OK) {
		lbm_set_esp_error_reason(res);
		return ENC_SYM_EERROR;
	}

	return touch_load_driver(driver, sym_cst816s, width, height);
}

static lbm_value ext_touch_load_gt911(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_RANGE(6, 7);

	int pin_sda = lbm_dec_as_i32(args[0]);
	int pin_scl = lbm_dec_as_i32(args[1]);
	int pin_rst = lbm_dec_as_i32(args[2]);
	int pin_int = lbm_dec_as_i32(args[3]);
	int width = lbm_dec_as_i32(args[4]);
	int height = lbm_dec_as_i32(args[5]);
	int freq = TOUCH_I2C_DEFAULT_FREQ;

	if (argn == 7) {
		freq = lbm_dec_as_i32(args[6]);
	}

	if (!touch_validate_i2c_load_args(pin_sda, pin_scl, pin_rst, pin_int, width, height, freq)) {
		return ENC_SYM_EERROR;
	}

	if (!touch_runtime_init()) {
		lbm_set_error_reason(msg_touch_runtime);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	touch_delete_locked();
	xSemaphoreGive(touch_mutex);

	lispif_touch_driver_t driver = {0};
	esp_err_t res = touch_init_gt911_esp_lcd(
			pin_sda,
			pin_scl,
			pin_rst,
			pin_int,
			(uint16_t)width,
			(uint16_t)height,
			(uint32_t)freq,
			&driver);
	if (res != ESP_OK) {
		lbm_set_esp_error_reason(res);
		return ENC_SYM_EERROR;
	}

	return touch_load_driver(driver, sym_gt911, width, height);
}

static lbm_value ext_touch_load_cst9217(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_RANGE(6, 7);

	int pin_sda = lbm_dec_as_i32(args[0]);
	int pin_scl = lbm_dec_as_i32(args[1]);
	int pin_rst = lbm_dec_as_i32(args[2]);
	int pin_int = lbm_dec_as_i32(args[3]);
	int width = lbm_dec_as_i32(args[4]);
	int height = lbm_dec_as_i32(args[5]);
	int freq = TOUCH_I2C_DEFAULT_FREQ;

	if (argn == 7) {
		freq = lbm_dec_as_i32(args[6]);
	}

	if (!touch_validate_i2c_load_args(pin_sda, pin_scl, pin_rst, pin_int, width, height, freq)) {
		return ENC_SYM_EERROR;
	}

	if (!touch_runtime_init()) {
		lbm_set_error_reason(msg_touch_runtime);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	touch_delete_locked();
	xSemaphoreGive(touch_mutex);

	lispif_touch_driver_t driver = {0};
	esp_err_t res = touch_init_cst9217_esp_lcd(
			pin_sda,
			pin_scl,
			pin_rst,
			pin_int,
			(uint16_t)width,
			(uint16_t)height,
			(uint32_t)freq,
			&driver);
	if (res != ESP_OK) {
		lbm_set_esp_error_reason(res);
		return ENC_SYM_EERROR;
	}

	return touch_load_driver(driver, sym_cst9217, width, height);
}

static lbm_value ext_touch_load_xpt2046(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_RANGE(8, 9);

	int host = lbm_dec_as_i32(args[0]);
	int pin_mosi = lbm_dec_as_i32(args[1]);
	int pin_miso = lbm_dec_as_i32(args[2]);
	int pin_sclk = lbm_dec_as_i32(args[3]);
	int pin_cs = lbm_dec_as_i32(args[4]);
	int pin_int = lbm_dec_as_i32(args[5]);
	int width = lbm_dec_as_i32(args[6]);
	int height = lbm_dec_as_i32(args[7]);
	int freq = TOUCH_I2C_DEFAULT_FREQ;

	if (argn == 9) {
		freq = lbm_dec_as_i32(args[8]);
	}

	bool valid_host = host == (int)SPI2_HOST;
#ifdef SPI3_HOST
	valid_host = valid_host || host == (int)SPI3_HOST;
#endif
	if (!valid_host) {
		lbm_set_error_reason(msg_invalid_spi_host);
		return ENC_SYM_EERROR;
	}

	if (!utils_gpio_is_valid(pin_mosi) ||
			!utils_gpio_is_valid(pin_miso) ||
			!utils_gpio_is_valid(pin_sclk) ||
			!utils_gpio_is_valid(pin_cs) ||
			!touch_gpio_is_valid_or_nc(pin_int)) {
		lbm_set_error_reason(msg_invalid_gpio);
		return ENC_SYM_EERROR;
	}

	if (width <= 0 || height <= 0) {
		lbm_set_error_reason(msg_invalid_size);
		return ENC_SYM_EERROR;
	}

	if (freq <= 0) {
		lbm_set_error_reason(msg_invalid_i2c_speed);
		return ENC_SYM_EERROR;
	}

	if (!touch_runtime_init()) {
		lbm_set_error_reason(msg_touch_runtime);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	touch_delete_locked();
	xSemaphoreGive(touch_mutex);

	lispif_touch_driver_t driver = {0};
	esp_err_t res = touch_init_xpt2046_esp_lcd(
			host,
			pin_mosi,
			pin_miso,
			pin_sclk,
			pin_cs,
			pin_int,
			(uint16_t)width,
			(uint16_t)height,
			(uint32_t)freq,
			&driver);
	if (res != ESP_OK) {
		lbm_set_esp_error_reason(res);
		return ENC_SYM_EERROR;
	}

	return touch_load_driver(driver, sym_xpt2046, width, height);
}

static lbm_value ext_touch_read(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(0);

	if (!touch_lock_loaded_or_error()) {
		return ENC_SYM_EERROR;
	}

	esp_err_t read_res = touch_driver.read_data();
	if (read_res != ESP_OK) {
		xSemaphoreGive(touch_mutex);
		lbm_set_esp_error_reason(read_res);
		return ENC_SYM_EERROR;
	}

	lbm_value res = touch_get_point_locked();
	xSemaphoreGive(touch_mutex);
	return res;
}

static lbm_value ext_touch_delete(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(0);

	if (!touch_mutex) {
		return ENC_SYM_TRUE;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	touch_delete_locked();
	xSemaphoreGive(touch_mutex);

	return ENC_SYM_TRUE;
}

static lbm_value ext_touch_apply_transforms(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(3);

	if (!touch_lock_loaded_or_error()) {
		return ENC_SYM_EERROR;
	}

	bool swap_xy = lbm_dec_as_i32(args[0]) != 0;
	bool mirror_x = lbm_dec_as_i32(args[1]) != 0;
	bool mirror_y = lbm_dec_as_i32(args[2]) != 0;
	bool apply_mirror_x = mirror_x;
	bool apply_mirror_y = mirror_y;

	if (swap_xy && touch_handle && touch_handle->set_swap_xy == NULL) {
		apply_mirror_x = mirror_y;
		apply_mirror_y = mirror_x;
	}

	esp_err_t res = esp_lcd_touch_set_swap_xy(touch_handle, swap_xy);
	if (res == ESP_OK) {
		res = esp_lcd_touch_set_mirror_x(touch_handle, apply_mirror_x);
	}
	if (res == ESP_OK) {
		res = esp_lcd_touch_set_mirror_y(touch_handle, apply_mirror_y);
	}

	if (res != ESP_OK) {
		xSemaphoreGive(touch_mutex);
		lbm_set_esp_error_reason(res);
		return ENC_SYM_EERROR;
	}

	xSemaphoreGive(touch_mutex);
	return ENC_SYM_TRUE;
}

void lispif_load_touch_extensions(void) {
	lbm_add_symbol_const("cst816s", &sym_cst816s);
	lbm_add_symbol_const("gt911", &sym_gt911);
	lbm_add_symbol_const("cst9217", &sym_cst9217);
	lbm_add_symbol_const("xpt2046", &sym_xpt2046);

	lbm_add_extension("touch-load-cst816s", ext_touch_load_cst816s);
	lbm_add_extension("touch-load-gt911", ext_touch_load_gt911);
	lbm_add_extension("touch-load-cst9217", ext_touch_load_cst9217);
	lbm_add_extension("touch-load-xpt2046", ext_touch_load_xpt2046);
	lbm_add_extension("touch-read", ext_touch_read);
	lbm_add_extension("touch-delete", ext_touch_delete);
	lbm_add_extension("touch-apply-transforms", ext_touch_apply_transforms);
}