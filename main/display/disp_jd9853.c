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

#include "disp_jd9853.h"
#include "hwspi.h"
#include "lispif.h"
#include "lispbm.h"

static int display_width = 172;
static int display_height = 320;
static int x_gap = 34;
static int y_gap = 0;

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

#define COMMAND()	(DISP_REG_CLR = 1 << m_pin_dc)
#define DATA()		(DISP_REG_SET = 1 << m_pin_dc)

typedef struct {
	uint8_t cmd;
	const uint8_t *data;
	uint8_t data_bytes;
	uint16_t delay_ms;
} jd9853_init_cmd_t;

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

static const jd9853_init_cmd_t init_cmds[] = {
	{0x11, NULL, 0, 120},
	{0xDF, (const uint8_t[]){0x98, 0x53}, 2, 0},
	{0xDF, (const uint8_t[]){0x98, 0x53}, 2, 0},
	{0xB2, (const uint8_t[]){0x23}, 1, 0},
	{0xB7, (const uint8_t[]){0x00, 0x47, 0x00, 0x6F}, 4, 0},
	{0xBB, (const uint8_t[]){0x1C, 0x1A, 0x55, 0x73, 0x63, 0xF0}, 6, 0},
	{0xC0, (const uint8_t[]){0x44, 0xA4}, 2, 0},
	{0xC1, (const uint8_t[]){0x16}, 1, 0},
	{0xC3, (const uint8_t[]){0x7D, 0x07, 0x14, 0x06, 0xCF, 0x71, 0x72, 0x77}, 8, 0},
	{0xC4, (const uint8_t[]){0x00, 0x00, 0xA0, 0x79, 0x0B, 0x0A, 0x16, 0x79, 0x0B, 0x0A, 0x16, 0x82}, 12, 0},
	{0xC8, (const uint8_t[]){0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00, 0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00}, 32, 0},
	{0xD0, (const uint8_t[]){0x04, 0x06, 0x6B, 0x0F, 0x00}, 5, 0},
	{0xD7, (const uint8_t[]){0x00, 0x30}, 2, 0},
	{0xE6, (const uint8_t[]){0x14}, 1, 0},
	{0xDE, (const uint8_t[]){0x01}, 1, 0},
	{0xB7, (const uint8_t[]){0x03, 0x13, 0xEF, 0x35, 0x35}, 5, 0},
	{0xC1, (const uint8_t[]){0x14, 0x15, 0xC0}, 3, 0},
	{0xC2, (const uint8_t[]){0x06, 0x3A}, 2, 0},
	{0xC4, (const uint8_t[]){0x72, 0x12}, 2, 0},
	{0xBE, (const uint8_t[]){0x00}, 1, 0},
	{0xDE, (const uint8_t[]){0x02}, 1, 0},
	{0xE5, (const uint8_t[]){0x00, 0x02, 0x00}, 3, 0},
	{0xE5, (const uint8_t[]){0x01, 0x02, 0x00}, 3, 0},
	{0xDE, (const uint8_t[]){0x00}, 1, 0},
	{0x35, (const uint8_t[]){0x00}, 1, 0},
	{0x3A, (const uint8_t[]){0x05}, 1, 0},
	{0x2A, (const uint8_t[]){0x00, 0x22, 0x00, 0xCD}, 4, 0},
	{0x2B, (const uint8_t[]){0x00, 0x00, 0x01, 0x3F}, 4, 0},
	{0xDE, (const uint8_t[]){0x02}, 1, 0},
	{0xE5, (const uint8_t[]){0x00, 0x02, 0x00}, 3, 0},
	{0xDE, (const uint8_t[]){0x00}, 1, 0},
	{0x36, (const uint8_t[]){0x48}, 1, 0},
	{0x21, NULL, 0, 10},
	{0x29, NULL, 0, 50},
};

void disp_jd9853_command(uint8_t command, const uint8_t *args, int argn) {
	hwspi_begin();
	command_start(command);
	if (args != NULL && argn > 0) {
		hwspi_send_data(args, argn);
	}
	hwspi_end();
}

bool disp_jd9853_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) {
	if ((x + img->width) > display_width || (y + img->height) > display_height) {
		return false;
	}

	uint16_t cs = x + x_gap;
	uint16_t ce = cs + img->width - 1;
	uint16_t ps = y + y_gap;
	uint16_t pe = ps + img->height - 1;

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

	disp_jd9853_command(0x2A, col, 4);
	disp_jd9853_command(0x2B, row, 4);

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

void disp_jd9853_clear(uint32_t color) {
	uint16_t clear_color_disp = to_disp_color(color);

	uint16_t cs = x_gap;
	uint16_t ce = x_gap + display_width - 1;
	uint16_t ps = y_gap;
	uint16_t pe = y_gap + display_height - 1;

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

	disp_jd9853_command(0x2A, col, 4);
	disp_jd9853_command(0x2B, row, 4);

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

		disp_jd9853_command(cmd, paras, n);
		return ENC_SYM_TRUE;
	} else if (argn == 1) {
		uint8_t cmd = (uint8_t)lbm_dec_as_u32(args[0]);
		disp_jd9853_command(cmd, NULL, 0);
		return ENC_SYM_TRUE;
	}

	return ENC_SYM_TERROR;
}

static lbm_value ext_disp_orientation(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint8_t arg = 0;
	switch (lbm_dec_as_u32(args[0])) {
	case 0:
		arg = 0x08;
		display_width = 172;
		display_height = 320;
		x_gap = 34;
		y_gap = 0;
		break;
	case 1:
		arg = 0x68;
		display_width = 320;
		display_height = 172;
		x_gap = 0;
		y_gap = 34;
		break;
	case 2:
		arg = 0xC8;
		display_width = 172;
		display_height = 320;
		x_gap = 34;
		y_gap = 0;
		break;
	case 3:
		arg = 0xA8;
		display_width = 320;
		display_height = 172;
		x_gap = 0;
		y_gap = 34;
		break;
	default:
		return ENC_SYM_EERROR;
	}

	disp_jd9853_command(0x36, &arg, 1);
	return ENC_SYM_TRUE;
}

void disp_jd9853_init(int pin_sd0, int pin_clk, int pin_cs, int pin_reset, int pin_dc, int clock_mhz) {
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

void disp_jd9853_reset(void) {
	if (m_pin_reset >= 0) {
		gpio_set_level(m_pin_reset, 0);
		vTaskDelay(pdMS_TO_TICKS(10));
		gpio_set_level(m_pin_reset, 1);
		vTaskDelay(pdMS_TO_TICKS(120));
	} else {
		disp_jd9853_command(0x01, NULL, 0);
		vTaskDelay(pdMS_TO_TICKS(120));
	}

	for (unsigned int i = 0; i < sizeof(init_cmds) / sizeof(init_cmds[0]); i++) {
		disp_jd9853_command(init_cmds[i].cmd, init_cmds[i].data, init_cmds[i].data_bytes);
		if (init_cmds[i].delay_ms > 0) {
			vTaskDelay(pdMS_TO_TICKS(init_cmds[i].delay_ms));
		}
	}

	display_width = 172;
	display_height = 320;
	x_gap = 34;
	y_gap = 0;
	disp_jd9853_clear(0);
}
