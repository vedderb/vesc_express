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

#include "hw_dash35s.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/ppa.h"
#include "esp_heap_caps.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_io.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lispif.h"
#include "lispif_disp_extensions.h"
#include "extensions.h"
#include "heap.h"

// Display
#define DISP_CS                     44
#define DISP_DC                     43
#define DISP_WR                     42
#define DISP_RD                     40
#define DISP_RESET                  6
#define DISP_IM0                    2
#define DISP_FMARK                  5
#define DISP_BL                     4
#define DISP_DATA_PINS              {7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 35, 38, 39}
#define DISP_CLOCK_HZ               10000000

#define DISPLAY_PIXELS             (320 * 480)

static esp_lcd_panel_io_handle_t display_io;
static ppa_client_handle_t fill_client;
static SemaphoreHandle_t display_mutex;
static SemaphoreHandle_t transfer_done;
static SemaphoreHandle_t frame_ready;
static bool tearing_enabled;
static uint16_t *framebuffer;
static uint16_t *transfer_buffer;
static bool transfer_pending;
static unsigned display_width = 480;
static unsigned display_height = 320;
static unsigned orientation = 3;

static bool transfer_finished(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *event, void *ctx) {
	BaseType_t wake = pdFALSE;
	xSemaphoreGiveFromISR(transfer_done, &wake);
	return wake == pdTRUE;
}

static void frame_mark(void *arg) {
	BaseType_t wake = pdFALSE;
	xSemaphoreGiveFromISR(frame_ready, &wake);
	if (wake == pdTRUE) {
		portYIELD_FROM_ISR();
	}
}

static void wait_transfer(void) {
	if (!transfer_pending) {
		return;
	}

	xSemaphoreTake(transfer_done, portMAX_DELAY);
	transfer_pending = false;
}

static bool command(uint8_t cmd, const void *data, size_t bytes) {
	wait_transfer();
	return esp_lcd_panel_io_tx_param(display_io, cmd, data, bytes) == ESP_OK;
}

static bool write_region(unsigned x, unsigned y, unsigned width, unsigned height) {
	unsigned right = (x + width + 63) / 64 * 64;
	unsigned bottom = y + height;
	x = x / 64 * 64;

	if (right > display_width) {
		x = 0;
		right = display_width;
		unsigned alignment = display_width == 480 ? 2 : 1;
		y = y / alignment * alignment;
		bottom = (bottom + alignment - 1) / alignment * alignment;
	}

	width = right - x;
	height = bottom - y;
	wait_transfer();

	for (unsigned row = 0; row < height; row++) {
		memcpy(transfer_buffer + row * width, framebuffer + (y + row) * display_width + x, width * 2);
	}

	if (tearing_enabled && !gpio_get_level(DISP_FMARK)) {
		xSemaphoreTake(frame_ready, 0);
		xSemaphoreTake(frame_ready, pdMS_TO_TICKS(25));
	}

	uint8_t col[] = {x >> 8, x, (right - 1) >> 8, right - 1};
	uint8_t row[] = {y >> 8, y, (bottom - 1) >> 8, bottom - 1};
	if (!command(0x2a, col, sizeof(col)) || !command(0x2b, row, sizeof(row)) ||
		esp_lcd_panel_io_tx_color(display_io, 0x2c, transfer_buffer, width * height * 2) != ESP_OK) {
		return false;
	}

	transfer_pending = true;
	return true;
}

static bool write_rows(unsigned first, unsigned last) {
	return write_region(0, first, display_width, last - first);
}

static uint16_t rgb565_color(uint32_t color) {
	uint16_t pixel = ((color >> 8) & 0xf800) | ((color >> 5) & 0x07e0) | ((color >> 3) & 0x001f);
	return __builtin_bswap16(pixel);
}

static void blast_rgb888(const uint8_t *data, uint16_t *pixels, unsigned count) {
	for (unsigned i = 0; i < count; i++) {
		uint16_t pixel = ((data[3 * i] & 0xf8) << 8) | ((data[3 * i + 1] & 0xfc) << 3) | (data[3 * i + 2] >> 3);
		pixels[i] = __builtin_bswap16(pixel);
	}
}

static void blast_rgb332(const uint8_t *data, uint16_t *pixels, unsigned count) {
	for (unsigned i = 0; i < count; i++) {
		unsigned r = data[i] >> 5;
		unsigned g = (data[i] >> 2) & 7;
		unsigned b = data[i] & 3;
		r = (r << 2) | (r >> 1);
		g = (g << 3) | g;
		b = b ? 2 * b + 1 : 0;
		b = (b << 2) | (b >> 1);
		uint16_t pixel = (r << 11) | (g << 5) | b;
		pixels[i] = __builtin_bswap16(pixel);
	}
}

static void blast_indexed(image_buffer_t *img, color_t *colors, unsigned row, uint16_t *pixels) {
	unsigned bits = img->fmt;
	unsigned mask = (1u << bits) - 1;
	size_t first = row * img->width;

	for (unsigned x = 0; x < img->width; x++) {
		size_t bit = (first + x) * bits;
		unsigned index = (img->data[bit >> 3] >> (8 - bits - (bit & 7))) & mask;
		uint32_t color = COLOR_TO_RGB888(colors[index], x, row);
		pixels[x] = rgb565_color(color);
	}
}

static bool render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) {
	if (!img || !img->data || !img->width || !img->height) {
		return false;
	}
	if (img->fmt != indexed2 && img->fmt != indexed4 && img->fmt != indexed16 && img->fmt != rgb332 &&
		img->fmt != rgb565 && img->fmt != rgb888) {
		return false;
	}
	if (img->fmt <= indexed16 && !colors) {
		return false;
	}

	xSemaphoreTake(display_mutex, portMAX_DELAY);
	if (x + img->width > display_width || y + img->height > display_height) {
		xSemaphoreGive(display_mutex);
		return false;
	}

	if (img->fmt == rgb565 && img->width == display_width) {
		memcpy(framebuffer + y * display_width, img->data, img->width * img->height * 2);
		bool ok = write_region(x, y, img->width, img->height);
		xSemaphoreGive(display_mutex);
		return ok;
	}

	for (unsigned row = 0; row < img->height; row++) {
		size_t first = row * img->width;
		uint16_t *pixels = framebuffer + (y + row) * display_width + x;

		switch (img->fmt) {
		case rgb565:
			memcpy(pixels, img->data + first * 2, img->width * 2);
			break;
		case rgb888:
			blast_rgb888(img->data + first * 3, pixels, img->width);
			break;
		case rgb332:
			blast_rgb332(img->data + first, pixels, img->width);
			break;
		default:
			blast_indexed(img, colors, row, pixels);
			break;
		}
	}

	bool ok = write_region(x, y, img->width, img->height);
	xSemaphoreGive(display_mutex);
	return ok;
}

static void fill_framebuffer(uint32_t color) {
	uint16_t pixel = rgb565_color(color);
	ppa_fill_oper_config_t fill = {
		.out = {
			.buffer = framebuffer,
			.buffer_size = DISPLAY_PIXELS * 2,
			.pic_w = display_width,
			.pic_h = display_height,
			.fill_cm = PPA_FILL_COLOR_MODE_RGB565,
		},
		.fill_block_w = display_width,
		.fill_block_h = display_height,
		.fill_color_val = 0xff000000u | ((pixel & 0xf800u) << 8) |
			((pixel & 0x07e0u) << 5) | ((pixel & 0x001fu) << 3),
		.mode = PPA_TRANS_MODE_BLOCKING,
	};
	if (fill_client && ppa_do_fill(fill_client, &fill) == ESP_OK) {
		return;
	}

	for (size_t i = 0; i < DISPLAY_PIXELS; i++) {
		framebuffer[i] = pixel;
	}
}

static void clear_display(uint32_t color) {
	xSemaphoreTake(display_mutex, portMAX_DELAY);
	fill_framebuffer(color);
	write_rows(0, display_height);
	xSemaphoreGive(display_mutex);
}

static bool set_orientation(unsigned value) {
	static const uint8_t modes[] = {0x48, 0xe8, 0x88, 0x28};
	if (value > 3 || !command(0x36, &modes[value], 1)) {
		return false;
	}

	orientation = value;
	display_width = value & 1 ? 480 : 320;
	display_height = value & 1 ? 320 : 480;
	return true;
}

static const uint8_t init_cmds[][16] = {
	{2, 0xf0, 0xc3},
	{2, 0xf0, 0x96},
	{2, 0xb4, 0x01},
	{2, 0xb7, 0xc6},
	{9, 0xe8, 0x40, 0x8a, 0x00, 0x00, 0x29, 0x19, 0xa5, 0x33},
	{2, 0xc1, 0x00},
	{2, 0xc2, 0xa7},
	{2, 0xc5, 0x08},
	{15, 0xe0, 0xf0, 0x06, 0x0b, 0x07, 0x06, 0x05, 0x2e, 0x33, 0x47, 0x3a, 0x17, 0x16, 0x2e, 0x31},
	{15, 0xe1, 0xf0, 0x09, 0x0d, 0x09, 0x08, 0x23, 0x2e, 0x33, 0x46, 0x38, 0x13, 0x13, 0x2c, 0x32},
	{2, 0xf0, 0x3c},
	{2, 0xf0, 0x69},
};

static void reset_display(void) {
	xSemaphoreTake(display_mutex, portMAX_DELAY);
	wait_transfer();
	tearing_enabled = false;
	gpio_set_level(DISP_RESET, 0);
	vTaskDelay(pdMS_TO_TICKS(100));
	gpio_set_level(DISP_RESET, 1);
	vTaskDelay(pdMS_TO_TICKS(220));

	ESP_ERROR_CHECK(command(0x11, NULL, 0) ? ESP_OK : ESP_FAIL);
	vTaskDelay(pdMS_TO_TICKS(220));
	
	for (unsigned i = 0; i < sizeof(init_cmds) / sizeof(init_cmds[0]); i++) {
		ESP_ERROR_CHECK(command(init_cmds[i][1], &init_cmds[i][2], init_cmds[i][0] - 1) ? ESP_OK : ESP_FAIL);
	}

	uint8_t format = 0x05;
	ESP_ERROR_CHECK(command(0x3a, &format, 1) ? ESP_OK : ESP_FAIL);
	ESP_ERROR_CHECK(set_orientation(orientation) ? ESP_OK : ESP_FAIL);
	ESP_ERROR_CHECK(command(0x21, NULL, 0) ? ESP_OK : ESP_FAIL);
	vTaskDelay(pdMS_TO_TICKS(120));

	fill_framebuffer(0);
	ESP_ERROR_CHECK(write_rows(0, display_height) ? ESP_OK : ESP_FAIL);
	ESP_ERROR_CHECK(command(0x29, NULL, 0) ? ESP_OK : ESP_FAIL);
	xSemaphoreGive(display_mutex);
}

static lbm_value ext_backlight(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	float percent = lbm_dec_as_float(args[0]);

	if (!(percent >= 0.0f && percent <= 100.0f)) {
		return ENC_SYM_EERROR;
	}

	uint32_t duty = percent * 1023.0f / 100.0f + 0.5f;
	esp_err_t res = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
	if (res == ESP_OK) {
		res = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
	}

	return res == ESP_OK ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

static lbm_value ext_orientation(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	xSemaphoreTake(display_mutex, portMAX_DELAY);
	bool ok = set_orientation(lbm_dec_as_u32(args[0]));
	xSemaphoreGive(display_mutex);

	return ok ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

static lbm_value ext_command(lbm_value *args, lbm_uint argn) {
	if (argn < 1 || argn > 17) {
		return ENC_SYM_EERROR;
	}
	LBM_CHECK_NUMBER_ALL();

	uint8_t data[16];

	for (lbm_uint i = 1; i < argn; i++) {
		data[i - 1] = lbm_dec_as_u32(args[i]);
	}

	xSemaphoreTake(display_mutex, portMAX_DELAY);
	bool ok = command(lbm_dec_as_u32(args[0]), data, argn - 1);
	xSemaphoreGive(display_mutex);

	return ok ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

static void button_pulls(bool enabled) {
	gpio_set_level(BTN1_PULL_GPIO, 1);
	gpio_set_level(BTN2_PULL_GPIO, 1);
	gpio_set_direction(BTN1_PULL_GPIO, enabled ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT);
	gpio_set_direction(BTN2_PULL_GPIO, enabled ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT);
}

static lbm_value ext_button_pulls(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);
	button_pulls(!lbm_is_symbol_nil(args[0]));

	return ENC_SYM_TRUE;
}

static lbm_value ext_button_read(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(0);
	int first = adc_get_raw(HW_ADC_CH0);
	int second = adc_get_raw(HW_ADC_CH1);

	if (first < 0 || second < 0) {
		return ENC_SYM_EERROR;
	}

	return lbm_heap_allocate_list_init(2, lbm_enc_i(first), lbm_enc_i(second));
}

static void load_extensions(bool main_found) {
	lbm_display_extensions_set_callbacks(render_image, clear_display, reset_display);

	if (main_found) {
		return;
	}

	lbm_add_extension("disp-set-bl", ext_backlight);
	lbm_add_extension("disp-orientation", ext_orientation);
	lbm_add_extension("disp-cmd", ext_command);
	lbm_add_extension("btn-pull-en", ext_button_pulls);
	lbm_add_extension("btn-read", ext_button_read);
}

void hw_init(void) {
	display_mutex = xSemaphoreCreateMutex();
	transfer_done = xSemaphoreCreateBinary();
	frame_ready = xSemaphoreCreateBinary();
	ESP_ERROR_CHECK(display_mutex && transfer_done && frame_ready ? ESP_OK : ESP_ERR_NO_MEM);

	gpio_config_t pins = {
		.pin_bit_mask = BIT64(DISP_RESET) | BIT64(DISP_RD) | BIT64(DISP_IM0),
		.mode = GPIO_MODE_OUTPUT,
	};

	ESP_ERROR_CHECK(gpio_config(&pins));
	gpio_set_level(DISP_RESET, 1);
	gpio_set_level(DISP_RD, 1);
	gpio_set_level(DISP_IM0, 0);
	button_pulls(false);

	gpio_config_t fmark = {
		.pin_bit_mask = BIT64(DISP_FMARK),
		.mode = GPIO_MODE_INPUT,
		.pull_down_en = GPIO_PULLDOWN_ENABLE,
		.intr_type = GPIO_INTR_POSEDGE,
	};
	ESP_ERROR_CHECK(gpio_config(&fmark));

	esp_err_t isr = gpio_install_isr_service(0);
	ESP_ERROR_CHECK(isr == ESP_ERR_INVALID_STATE ? ESP_OK : isr);
	ESP_ERROR_CHECK(gpio_isr_handler_add(DISP_FMARK, frame_mark, NULL));

	ledc_timer_config_t timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = LEDC_TIMER_10_BIT,
		.timer_num = LEDC_TIMER_0,
		.freq_hz = 20000,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ESP_ERROR_CHECK(ledc_timer_config(&timer));

	ledc_channel_config_t backlight = {
		.gpio_num = DISP_BL,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_0,
		.timer_sel = LEDC_TIMER_0,
		.duty = 0,
	};
	ESP_ERROR_CHECK(ledc_channel_config(&backlight));

	esp_lcd_i80_bus_config_t bus_config = {
		.dc_gpio_num = DISP_DC,
		.wr_gpio_num = DISP_WR,
		.clk_src = LCD_CLK_SRC_DEFAULT,
		.data_gpio_nums = DISP_DATA_PINS,
		.bus_width = 16,
		.max_transfer_bytes = DISPLAY_PIXELS * 2,
	};

	esp_lcd_i80_bus_handle_t bus;
	ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &bus));

	esp_lcd_panel_io_i80_config_t io_config = {
		.cs_gpio_num = DISP_CS,
		.pclk_hz = DISP_CLOCK_HZ,
		.trans_queue_depth = 1,
		.on_color_trans_done = transfer_finished,
		.lcd_cmd_bits = 8,
		.lcd_param_bits = 8,
		.dc_levels = {.dc_idle_level = 1, .dc_data_level = 1},
		.flags.swap_color_bytes = true,
	};
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(bus, &io_config, &display_io));

	framebuffer = heap_caps_aligned_calloc(128, DISPLAY_PIXELS, 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
	ESP_ERROR_CHECK(framebuffer ? ESP_OK : ESP_ERR_NO_MEM);

	transfer_buffer = heap_caps_aligned_calloc(128, DISPLAY_PIXELS, 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
	ESP_ERROR_CHECK(transfer_buffer ? ESP_OK : ESP_ERR_NO_MEM);

	ppa_client_config_t fill_config = {.oper_type = PPA_OPERATION_FILL};
	ESP_ERROR_CHECK(ppa_register_client(&fill_config, &fill_client));

	lispif_add_ext_load_callback(load_extensions);
}
