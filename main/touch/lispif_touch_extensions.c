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
#include "esp_err.h"

#include "touch_cst816s.h"
#include "touch_gt911.h"
#include "touch_xpt2046.h"

#define TOUCH_I2C_PORT I2C_NUM_0
#define TOUCH_I2C_DEFAULT_FREQ 400000
#define TOUCH_XPT2046_Z_THRESHOLD_DEFAULT 400
#define TOUCH_EVENT_TASK_STACK 2048

static char *msg_invalid_gpio = "Invalid GPIO";
static char *msg_invalid_size = "Invalid touch size";
static char *msg_invalid_i2c_speed = "Invalid I2C speed";
static char *msg_invalid_touch_addr = "Invalid touch address";
static char *msg_invalid_touch_threshold = "Invalid touch threshold";
static char *msg_touch_not_loaded = "Touch not loaded";
static char *msg_touch_runtime = "Touch runtime init failed";

static SemaphoreHandle_t touch_mutex = 0;
static TaskHandle_t touch_event_task_handle = 0;
static lispif_touch_driver_t touch_driver = {0};
static lbm_uint touch_driver_symbol = 0;
static uint16_t touch_width = 0;
static uint16_t touch_height = 0;
static bool touch_swap_xy = false;
static bool touch_mirror_x = false;
static bool touch_mirror_y = false;

static lbm_uint sym_cst816s = 0;
static lbm_uint sym_gt911 = 0;
static lbm_uint sym_xpt2046 = 0;

static void touch_event_task(void *arg);

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

static void touch_apply_transforms(lispif_touch_point_data_t *points, uint8_t point_cnt) {
	for (uint8_t i = 0;i < point_cnt;i++) {
		uint16_t x = points[i].x;
		uint16_t y = points[i].y;

		if (touch_mirror_x) {
			x = touch_width > 0 ? (touch_width - 1U) - x : x;
		}

		if (touch_mirror_y) {
			y = touch_height > 0 ? (touch_height - 1U) - y : y;
		}

		if (touch_swap_xy) {
			uint16_t tmp = x;
			x = y;
			y = tmp;
		}

		points[i].x = x;
		points[i].y = y;
	}
}

static void touch_delete_locked(void) {
	if (touch_driver.deinit) {
		touch_driver.deinit();
	}

	memset(&touch_driver, 0, sizeof(touch_driver));
	touch_driver_symbol = 0;
	touch_width = 0;
	touch_height = 0;
	touch_swap_xy = false;
	touch_mirror_x = false;
	touch_mirror_y = false;
}

static lbm_value touch_get_point_locked(void) {
	lispif_touch_point_data_t point;
	uint8_t point_cnt = 0;
	esp_err_t res = touch_driver.get_data(&point, &point_cnt, 1);
	if (res != ESP_OK) {
		lbm_set_esp_error_reason(res);
		return ENC_SYM_EERROR;
	}

	touch_apply_transforms(&point, point_cnt);

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
					touch_apply_transforms(&point, point_cnt);
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

	if (!utils_gpio_is_valid(pin_sda) ||
			!utils_gpio_is_valid(pin_scl) ||
			!touch_gpio_is_valid_or_nc(pin_rst) ||
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

	/* Tear down the previous instance before reconfiguring the shared I2C port.
	 * Stream reloads can call this repeatedly, and keeping the old panel IO/ISR
	 * alive during re-init makes the touch controller fail intermittently.
	 */
	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	touch_delete_locked();
	xSemaphoreGive(touch_mutex);
	touch_cst816s_config_t conf = {
			.pin_sda = pin_sda,
			.pin_scl = pin_scl,
			.pin_rst = pin_rst,
			.pin_int = pin_int,
			.i2c_port = TOUCH_I2C_PORT,
			.i2c_addr = TOUCH_CST816S_I2C_ADDR,
			.i2c_freq = (uint32_t)freq,
			.timeout_ms = 50,
			.reset_level = 0,
			.interrupt_level = 0,
			.read_id = true,
	};

	lispif_touch_driver_t driver = {0};
	esp_err_t res = touch_cst816s_init(&conf, &driver);
	if (res != ESP_OK) {
		lbm_set_esp_error_reason(res);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	touch_driver = driver;
	touch_driver_symbol = sym_cst816s;
	touch_width = (uint16_t)width;
	touch_height = (uint16_t)height;
	xSemaphoreGive(touch_mutex);

	return ENC_SYM_TRUE;
}

static lbm_value ext_touch_load_gt911(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_RANGE(6, 8);

	int pin_sda = lbm_dec_as_i32(args[0]);
	int pin_scl = lbm_dec_as_i32(args[1]);
	int pin_rst = lbm_dec_as_i32(args[2]);
	int pin_int = lbm_dec_as_i32(args[3]);
	int width = lbm_dec_as_i32(args[4]);
	int height = lbm_dec_as_i32(args[5]);
	int freq = TOUCH_I2C_DEFAULT_FREQ;
	int addr = TOUCH_GT911_I2C_ADDR_PRIMARY;

	if (argn >= 7) {
		freq = lbm_dec_as_i32(args[6]);
	}

	if (argn == 8) {
		addr = lbm_dec_as_i32(args[7]);
	}

	if (!utils_gpio_is_valid(pin_sda) ||
			!utils_gpio_is_valid(pin_scl) ||
			!touch_gpio_is_valid_or_nc(pin_rst) ||
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

	if (addr != TOUCH_GT911_I2C_ADDR_PRIMARY && addr != TOUCH_GT911_I2C_ADDR_BACKUP) {
		lbm_set_error_reason(msg_invalid_touch_addr);
		return ENC_SYM_EERROR;
	}

	if (!touch_runtime_init()) {
		lbm_set_error_reason(msg_touch_runtime);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	touch_delete_locked();
	xSemaphoreGive(touch_mutex);

	touch_gt911_config_t conf = {
			.pin_sda = pin_sda,
			.pin_scl = pin_scl,
			.pin_rst = pin_rst,
			.pin_int = pin_int,
			.i2c_port = TOUCH_I2C_PORT,
			.i2c_addr = (uint8_t)addr,
			.i2c_freq = (uint32_t)freq,
			.timeout_ms = 50,
			.reset_level = 0,
			.interrupt_level = 0,
			.read_cfg = true,
	};

	lispif_touch_driver_t driver = {0};
	esp_err_t res = touch_gt911_init(&conf, &driver);
	if (res != ESP_OK) {
		lbm_set_esp_error_reason(res);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	touch_driver = driver;
	touch_driver_symbol = sym_gt911;
	touch_width = (uint16_t)width;
	touch_height = (uint16_t)height;
	xSemaphoreGive(touch_mutex);

	return ENC_SYM_TRUE;
}

static lbm_value ext_touch_load_xpt2046(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_RANGE(7, 8);

	int pin_mosi = lbm_dec_as_i32(args[0]);
	int pin_miso = lbm_dec_as_i32(args[1]);
	int pin_clk = lbm_dec_as_i32(args[2]);
	int pin_cs = lbm_dec_as_i32(args[3]);
	int pin_int = lbm_dec_as_i32(args[4]);
	int width = lbm_dec_as_i32(args[5]);
	int height = lbm_dec_as_i32(args[6]);
	int z_threshold = TOUCH_XPT2046_Z_THRESHOLD_DEFAULT;

	if (argn == 8) {
		z_threshold = lbm_dec_as_i32(args[7]);
	}

	if (!utils_gpio_is_valid(pin_mosi) ||
			!utils_gpio_is_valid(pin_miso) ||
			!utils_gpio_is_valid(pin_clk) ||
			!utils_gpio_is_valid(pin_cs) ||
			!touch_gpio_is_valid_or_nc(pin_int)) {
		lbm_set_error_reason(msg_invalid_gpio);
		return ENC_SYM_EERROR;
	}

	if (width <= 0 || height <= 0) {
		lbm_set_error_reason(msg_invalid_size);
		return ENC_SYM_EERROR;
	}

	if (z_threshold <= 0) {
		lbm_set_error_reason(msg_invalid_touch_threshold);
		return ENC_SYM_EERROR;
	}

	if (!touch_runtime_init()) {
		lbm_set_error_reason(msg_touch_runtime);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	touch_delete_locked();
	xSemaphoreGive(touch_mutex);

	touch_xpt2046_config_t conf = {
			.pin_mosi = pin_mosi,
			.pin_miso = pin_miso,
			.pin_clk = pin_clk,
			.pin_cs = pin_cs,
			.pin_int = pin_int,
			.width = (uint16_t)width,
			.height = (uint16_t)height,
			.z_threshold = (uint16_t)z_threshold,
			.interrupt_level = 0,
	};

	lispif_touch_driver_t driver = {0};
	esp_err_t res = touch_xpt2046_init(&conf, &driver);
	if (res != ESP_OK) {
		lbm_set_esp_error_reason(res);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	touch_driver = driver;
	touch_driver_symbol = sym_xpt2046;
	touch_width = (uint16_t)width;
	touch_height = (uint16_t)height;
	xSemaphoreGive(touch_mutex);

	return ENC_SYM_TRUE;
}

static lbm_value ext_touch_read(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(0);

	if (!touch_mutex) {
		lbm_set_error_reason(msg_touch_not_loaded);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	if (!touch_driver_loaded()) {
		xSemaphoreGive(touch_mutex);
		lbm_set_error_reason(msg_touch_not_loaded);
		return ENC_SYM_EERROR;
	}

	esp_err_t res = touch_driver.read_data();
	xSemaphoreGive(touch_mutex);

	if (res != ESP_OK) {
		lbm_set_esp_error_reason(res);
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_touch_data(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(0);

	if (!touch_mutex) {
		lbm_set_error_reason(msg_touch_not_loaded);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	if (!touch_driver_loaded()) {
		xSemaphoreGive(touch_mutex);
		lbm_set_error_reason(msg_touch_not_loaded);
		return ENC_SYM_EERROR;
	}

	lbm_value res = touch_get_point_locked();
	xSemaphoreGive(touch_mutex);
	return res;
}

static lbm_value ext_touch_read_and_get(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(0);

	if (!touch_mutex) {
		lbm_set_error_reason(msg_touch_not_loaded);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	if (!touch_driver_loaded()) {
		xSemaphoreGive(touch_mutex);
		lbm_set_error_reason(msg_touch_not_loaded);
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

static lbm_value ext_touch_swap_xy(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	if (!touch_mutex) {
		lbm_set_error_reason(msg_touch_not_loaded);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	if (!touch_driver_loaded()) {
		xSemaphoreGive(touch_mutex);
		lbm_set_error_reason(msg_touch_not_loaded);
		return ENC_SYM_EERROR;
	}

	touch_swap_xy = lbm_dec_as_i32(args[0]) != 0;
	xSemaphoreGive(touch_mutex);

	return ENC_SYM_TRUE;
}

static lbm_value ext_touch_mirror_x(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	if (!touch_mutex) {
		lbm_set_error_reason(msg_touch_not_loaded);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	if (!touch_driver_loaded()) {
		xSemaphoreGive(touch_mutex);
		lbm_set_error_reason(msg_touch_not_loaded);
		return ENC_SYM_EERROR;
	}

	touch_mirror_x = lbm_dec_as_i32(args[0]) != 0;
	xSemaphoreGive(touch_mutex);

	return ENC_SYM_TRUE;
}

static lbm_value ext_touch_mirror_y(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	if (!touch_mutex) {
		lbm_set_error_reason(msg_touch_not_loaded);
		return ENC_SYM_EERROR;
	}

	xSemaphoreTake(touch_mutex, portMAX_DELAY);
	if (!touch_driver_loaded()) {
		xSemaphoreGive(touch_mutex);
		lbm_set_error_reason(msg_touch_not_loaded);
		return ENC_SYM_EERROR;
	}

	touch_mirror_y = lbm_dec_as_i32(args[0]) != 0;
	xSemaphoreGive(touch_mutex);

	return ENC_SYM_TRUE;
}

void lispif_load_touch_extensions(void) {
	if (!touch_runtime_init()) {
		return;
	}

	lbm_add_symbol_const("cst816s", &sym_cst816s);
	lbm_add_symbol_const("gt911", &sym_gt911);
	lbm_add_symbol_const("xpt2046", &sym_xpt2046);

	lbm_add_extension("touch-load-cst816s", ext_touch_load_cst816s);
	lbm_add_extension("touch-load-gt911", ext_touch_load_gt911);
	lbm_add_extension("touch-load-xpt2046", ext_touch_load_xpt2046);
	lbm_add_extension("touch-read", ext_touch_read);
	lbm_add_extension("touch-data", ext_touch_data);
	lbm_add_extension("touch-read-and-get", ext_touch_read_and_get);
	lbm_add_extension("touch-delete", ext_touch_delete);
	lbm_add_extension("touch-swap-xy", ext_touch_swap_xy);
	lbm_add_extension("touch-mirror-x", ext_touch_mirror_x);
	lbm_add_extension("touch-mirror-y", ext_touch_mirror_y);
}