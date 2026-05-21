/*
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"

#include "disp_gc9a01.h"
#include "hwspi.h"
#include "lispif.h"
#include "lispbm.h"

static int display_width = 240;
static int display_height = 240;

static int m_pin_reset = -1;
static int m_pin_dc = -1;

#if CONFIG_IDF_TARGET_ESP32S3
	#define DISP_REG_SET		GPIO.out_w1ts
	#define DISP_REG_CLR		GPIO.out_w1tc
#elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32P4
	#define DISP_REG_SET		GPIO.out_w1ts.val
	#define DISP_REG_CLR		GPIO.out_w1tc.val
#else
	#error "Unsupported target"
#endif

#define COMMAND() 	    (DISP_REG_CLR = 1 << m_pin_dc)
#define DATA() 	        (DISP_REG_SET = 1 << m_pin_dc)

typedef struct {
	uint8_t cmd;
	const uint8_t *data;
	uint8_t data_bytes;
	uint16_t delay_ms;
} gc9a01_init_cmd_t;

static void command_start(uint8_t cmd) {
	COMMAND();
	hwspi_send_data(&cmd, 1);
	DATA();
}

static uint16_t to_disp_color(uint32_t rgb) {
	uint8_t b = (uint8_t)rgb;
	uint8_t g = (uint8_t)(rgb >> 8);
	uint8_t r = (uint8_t)(rgb >> 16);
	r >>= 3;
	g >>= 2;
	b >>= 3;

	uint8_t color_high = 0;
	color_high = r << 3;
	color_high |= (g >> 3);

	uint8_t color_low = 0;
	color_low = g << 5;
	color_low |= b;

	return color_high | (((uint16_t)color_low) << 8);
}

static void blast_indexed2(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i++) {
		int byte = i >> 3;
		int bit = 7 - (i & 0x7);
		int color_ind = (data[byte] & (1 << bit)) >> bit;

		uint16_t c = to_disp_color(
				COLOR_TO_RGB888(colors[color_ind], i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_indexed4(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i++) {
		int byte = i >> 2;
		int bit = (3 - (i & 0x03)) * 2;
		int color_ind = (data[byte] & (0x03 << bit)) >> bit;

		uint16_t c = to_disp_color(
				COLOR_TO_RGB888(colors[color_ind], i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_indexed16(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i++) {
		int byte = i >> 1;
		int bit = (1 - (i & 0x01)) * 4;
		int color_ind = (data[byte] & (0x0F << bit)) >> bit;

		uint16_t c = to_disp_color(
				COLOR_TO_RGB888(colors[color_ind], i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_rgb332(uint8_t *data, uint32_t num_pix) {
	command_start(0x2C);
	hwspi_data_stream_start();

	for (uint32_t i = 0; i < num_pix; i++) {
		uint8_t pix = data[i];
		uint32_t r = (uint32_t)((pix >> 5) & 0x7);
		uint32_t g = (uint32_t)((pix >> 2) & 0x7);
		uint32_t b = (uint32_t)(pix & 0x3);
		uint32_t rgb888 = r << (16 + 5) | g << (8 + 5) | b << 6;
		uint16_t disp = to_disp_color(rgb888);
		hwspi_data_stream_write((uint8_t)disp);
		hwspi_data_stream_write((uint8_t)(disp >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_rgb565(uint8_t *data, uint32_t num_pix) {
	command_start(0x2C);
	hwspi_data_stream_start();

	for (uint32_t i = 0; i < num_pix; i++) {
		uint16_t pix = (((uint16_t)data[2 * i]) << 8) | ((uint16_t)data[2 * i + 1]);

		uint32_t r = (uint32_t)(pix >> 11);
		uint32_t g = (uint32_t)((pix >> 5) & 0x3F);
		uint32_t b = (uint32_t)(pix & 0x1F);
		uint32_t rgb888 = r << (16 + 3) | g << (8 + 2) | b << 3;
		uint16_t disp = to_disp_color(rgb888);

		hwspi_data_stream_write((uint8_t)disp);
		hwspi_data_stream_write((uint8_t)(disp >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_rgb888(uint8_t *data, uint32_t num_pix) {
	command_start(0x2C);
	hwspi_data_stream_start();

	for (uint32_t i = 0; i < num_pix; i++) {
		uint32_t r = data[3 * i];
		uint32_t g = data[3 * i + 1];
		uint32_t b = data[3 * i + 2];

		uint32_t rgb888 = r << 16 | g << 8 | b;
		uint16_t disp = to_disp_color(rgb888);

		hwspi_data_stream_write((uint8_t)disp);
		hwspi_data_stream_write((uint8_t)(disp >> 8));
	}

	hwspi_data_stream_finish();
}

bool disp_gc9a01_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) {
	uint16_t cs = x;
	uint16_t ce = x + img->width - 1;
	uint16_t ps = y;
	uint16_t pe = y + img->height - 1;

	if (ce >= display_width || pe >= display_height) {
		return false;
	}

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

	disp_gc9a01_command(0x2A, col, 4);
	disp_gc9a01_command(0x2B, row, 4);

	uint32_t num_pix = img->width * img->height;

	hwspi_begin();
	switch (img->fmt) {
	case indexed2:
		if (!colors) return false;
		blast_indexed2(img, colors);
		break;
	case indexed4:
		if (!colors) return false;
		blast_indexed4(img, colors);
		break;
	case indexed16:
		if (!colors) return false;
		blast_indexed16(img, colors);
		break;
	case rgb332:
		blast_rgb332(img->data, num_pix);
		break;
	case rgb565:
		blast_rgb565(img->data, num_pix);
		break;
	case rgb888:
		blast_rgb888(img->data, num_pix);
		break;
	default:
		break;
	}
	hwspi_end();

	return true;
}

void disp_gc9a01_clear(uint32_t color) {
	uint16_t clear_color_disp = to_disp_color(color);

	uint16_t cs = 0;
	uint16_t ce = display_width - 1;
	uint16_t ps = 0;
	uint16_t pe = display_height - 1;

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

	disp_gc9a01_command(0x2A, col, 4);
	disp_gc9a01_command(0x2B, row, 4);

	hwspi_begin();
	command_start(0x2C);
	hwspi_data_stream_start();
	for (int i = 0; i < (display_width * display_height); i++) {
		hwspi_data_stream_write((uint8_t)clear_color_disp);
		hwspi_data_stream_write((uint8_t)(clear_color_disp >> 8));
	}
	hwspi_data_stream_finish();
	hwspi_end();
}

static lbm_value ext_disp_cmd(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	if (argn > 1) {
		uint8_t cmd = (uint8_t)lbm_dec_as_u32(args[0]);
		uint8_t paras[32];
		int n = (int)argn - 1;
		if (n > 32) {
			n = 32;
		}

		for (int i = 0; i < n; i++) {
			paras[i] = (uint8_t)lbm_dec_as_u32(args[i + 1]);
		}

		disp_gc9a01_command(cmd, paras, n);
		return ENC_SYM_TRUE;
	} else if (argn == 1) {
		uint8_t cmd = (uint8_t)lbm_dec_as_u32(args[0]);
		disp_gc9a01_command(cmd, NULL, 0);
		return ENC_SYM_TRUE;
	}

	return ENC_SYM_TERROR;
}

static lbm_value ext_disp_orientation(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint8_t arg = 0;
	switch (lbm_dec_as_u32(args[0])) {
	case 0:
		arg = 0x48;
		break;
	case 1:
		arg = 0x28;
		break;
	case 2:
		arg = 0x88;
		break;
	case 3:
		arg = 0xE8;
		break;
	default:
		return ENC_SYM_EERROR;
	}

	disp_gc9a01_command(0x36, &arg, 1);
	display_width = 240;
	display_height = 240;
	return ENC_SYM_TRUE;
}

void disp_gc9a01_init(int pin_sd0, int pin_clk, int pin_cs, int pin_reset, int pin_dc, int clock_mhz) {
	hwspi_init(clock_mhz, 0, -1, pin_sd0, pin_clk, pin_cs);
	m_pin_reset = pin_reset;
	m_pin_dc = pin_dc;

	gpio_config_t gpconf = {0};
	gpconf.pin_bit_mask = BIT(m_pin_dc);
	if (m_pin_reset >= 0) {
		gpconf.pin_bit_mask |= BIT(m_pin_reset);
	}
	gpconf.mode = GPIO_MODE_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpconf.intr_type = GPIO_INTR_DISABLE;

	gpio_config(&gpconf);

	if (m_pin_reset >= 0) {
		gpio_set_level(m_pin_reset, 1);
	}
	gpio_set_level(m_pin_dc, 0);

	lbm_add_extension("ext-disp-cmd", ext_disp_cmd);
	lbm_add_extension("ext-disp-orientation", ext_disp_orientation);
}

void disp_gc9a01_command(uint8_t command, const uint8_t *args, int argn) {
	hwspi_begin();
	command_start(command);
	if (args != NULL && argn > 0) {
		hwspi_send_data(args, argn);
	}
	hwspi_end();
}

static const gc9a01_init_cmd_t init_cmds[] = {
	{0xFE, NULL, 0, 0},
	{0xEF, NULL, 0, 0},
	{0xEB, (const uint8_t[]){0x14}, 1, 0},
	{0x84, (const uint8_t[]){0x60}, 1, 0},
	{0x85, (const uint8_t[]){0xFF}, 1, 0},
	{0x86, (const uint8_t[]){0xFF}, 1, 0},
	{0x87, (const uint8_t[]){0xFF}, 1, 0},
	{0x8E, (const uint8_t[]){0xFF}, 1, 0},
	{0x8F, (const uint8_t[]){0xFF}, 1, 0},
	{0x88, (const uint8_t[]){0x0A}, 1, 0},
	{0x89, (const uint8_t[]){0x23}, 1, 0},
	{0x8A, (const uint8_t[]){0x00}, 1, 0},
	{0x8B, (const uint8_t[]){0x80}, 1, 0},
	{0x8C, (const uint8_t[]){0x01}, 1, 0},
	{0x8D, (const uint8_t[]){0x03}, 1, 0},
	{0x90, (const uint8_t[]){0x08, 0x08, 0x08, 0x08}, 4, 0},
	{0xFF, (const uint8_t[]){0x60, 0x01, 0x04}, 3, 0},
	{0xC3, (const uint8_t[]){0x13}, 1, 0},
	{0xC4, (const uint8_t[]){0x13}, 1, 0},
	{0xC9, (const uint8_t[]){0x22}, 1, 0},
	{0xBE, (const uint8_t[]){0x11}, 1, 0},
	{0xE1, (const uint8_t[]){0x10, 0x0E}, 2, 0},
	{0xDF, (const uint8_t[]){0x21, 0x0C, 0x02}, 3, 0},
	{0xF0, (const uint8_t[]){0x45, 0x09, 0x08, 0x08, 0x26, 0x2A}, 6, 0},
	{0xF1, (const uint8_t[]){0x43, 0x70, 0x72, 0x36, 0x37, 0x6F}, 6, 0},
	{0xF2, (const uint8_t[]){0x45, 0x09, 0x08, 0x08, 0x26, 0x2A}, 6, 0},
	{0xF3, (const uint8_t[]){0x43, 0x70, 0x72, 0x36, 0x37, 0x6F}, 6, 0},
	{0xED, (const uint8_t[]){0x1B, 0x0B}, 2, 0},
	{0xAE, (const uint8_t[]){0x77}, 1, 0},
	{0xCD, (const uint8_t[]){0x63}, 1, 0},
	{0x70, (const uint8_t[]){0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03}, 9, 0},
	{0xE8, (const uint8_t[]){0x34}, 1, 0},
	{0x60, (const uint8_t[]){0x38, 0x0B, 0x6D, 0x6D, 0x39, 0xF0, 0x6D, 0x6D}, 8, 0},
	{0x61, (const uint8_t[]){0x38, 0xF4, 0x6D, 0x6D, 0x38, 0xF7, 0x6D, 0x6D}, 8, 0},
	{0x62, (const uint8_t[]){0x38, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x38, 0x0F, 0x71, 0xEF, 0x70, 0x70}, 12, 0},
	{0x63, (const uint8_t[]){0x38, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x38, 0x13, 0x71, 0xF3, 0x70, 0x70}, 12, 0},
	{0x64, (const uint8_t[]){0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07}, 7, 0},
	{0x66, (const uint8_t[]){0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00}, 10, 0},
	{0x67, (const uint8_t[]){0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98}, 10, 0},
	{0x74, (const uint8_t[]){0x10, 0x45, 0x80, 0x00, 0x00, 0x4E, 0x00}, 7, 0},
	{0x98, (const uint8_t[]){0x3E, 0x07}, 2, 0},
	{0x99, (const uint8_t[]){0x3E, 0x07}, 2, 0},
	{0x36, (const uint8_t[]){0x48}, 1, 0},
	{0x3A, (const uint8_t[]){0x55}, 1, 0},
	{0x21, NULL, 0, 0},
};

void disp_gc9a01_reset(void) {
	if (m_pin_reset >= 0) {
		gpio_set_level(m_pin_reset, 0);
		vTaskDelay(10);
		gpio_set_level(m_pin_reset, 1);
		vTaskDelay(10);
	} else {
		disp_gc9a01_command(0x01, NULL, 0);
		vTaskDelay(20);
	}

	disp_gc9a01_command(0x11, NULL, 0);
	vTaskDelay(100);

	for (unsigned int i = 0; i < sizeof(init_cmds) / sizeof(init_cmds[0]); i++) {
		disp_gc9a01_command(init_cmds[i].cmd, init_cmds[i].data, init_cmds[i].data_bytes);
		if (init_cmds[i].delay_ms > 0) {
			vTaskDelay(pdMS_TO_TICKS(init_cmds[i].delay_ms));
		}
	}

	disp_gc9a01_command(0x29, NULL, 0);
	vTaskDelay(20);

	display_width = 240;
	display_height = 240;
	disp_gc9a01_clear(0);
}
