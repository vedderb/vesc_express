/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se
	Copyright 2023 Joel Svensson    svenssonjoel@yahoo.se

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

#include "disp_ssd1351.h"
#include "hwspi.h"
#include "lispif.h"
#include "lispbm.h"

static int display_width = 128;
static int display_height = 128;

// Private variables
static int m_pin_reset = -1;
static int m_pin_dc    = -1;

#if CONFIG_IDF_TARGET_ESP32S3
	#define DISP_REG_SET		GPIO.out_w1ts
	#define DISP_REG_CLR		GPIO.out_w1tc
#elif CONFIG_IDF_TARGET_ESP32
	#define DISP_REG_SET    GPIO_OUT_W1TS_REG
	#define DISP_REG_CLR    GPIO_OUT_W1TC_REG
#else
	#define DISP_REG_SET		GPIO.out_w1ts.val
	#define DISP_REG_CLR		GPIO.out_w1tc.val
#endif

#if CONFIG_IDF_TARGET_ESP32
	#define COMMAND() 	    REG_WRITE(DISP_REG_SET, (1 << m_pin_dc))
	#define DATA() 	        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << m_pin_dc))
#else
	#define COMMAND() 	    (DISP_REG_CLR = 1 << m_pin_dc)
	#define DATA() 	        (DISP_REG_SET = 1 << m_pin_dc)
#endif

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

	// the order of output is bit 7 - 0 : 15 - 8
	uint16_t color = color_high;
	color |= (((uint16_t)color_low) << 8);
	return color;
}

static void blast_indexed2(image_buffer_t *img, color_t *colors) {
	command_start(0x5C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 3;
		int bit  = 7 - (i & 0x7);
		int color_ind = (data[byte] & (1 << bit)) >> bit;

		uint16_t c = to_disp_color(
				COLOR_TO_RGB888(colors[color_ind],
						i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_indexed4(image_buffer_t *img, color_t *colors) {
	command_start(0x5C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 2;
		int bit = (3 - (i & 0x03)) * 2;
		int color_ind = (data[byte] & (0x03 << bit)) >> bit;

		uint16_t c = to_disp_color(
				COLOR_TO_RGB888(colors[color_ind],
						i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_indexed16(image_buffer_t *img, color_t *colors) {
	command_start(0x5C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 1;    // byte to access is pix / 2
		int bit = (1 - (i & 0x01)) * 4; // bit position to access within byte
		int color_ind = (data[byte] & (0x0F << bit)) >> bit; // extract 4 bit value.

		uint16_t c = to_disp_color(
				COLOR_TO_RGB888(colors[color_ind],
						i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}


static void blast_rgb332(uint8_t *data, uint32_t num_pix) {
	command_start(0x5C);
	hwspi_data_stream_start();

	for (int i = 0; i < num_pix; i ++) {
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
	command_start(0x5C);
	hwspi_data_stream_start();

	for (int i = 0; i < num_pix; i ++) {
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
	command_start(0x5C);
	hwspi_data_stream_start();

	for (int i = 0; i < num_pix; i ++) {
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

bool disp_ssd1351_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) {
	uint16_t cs = x;
	uint16_t ce = x + img->width - 1;
	uint16_t ps = y;
	uint16_t pe = y + img->height - 1;

	if (ce >= display_width || pe >= display_height) {
		return false;
	}

	uint8_t col[2] = {cs, ce};
	uint8_t row[2] = {ps, pe};

	disp_ssd1351_command(0x15, col, 2);
	disp_ssd1351_command(0x75, row, 2);

	uint32_t num_pix = img->width * img->height;

	hwspi_begin();
	switch(img->fmt) {
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

void disp_ssd1351_clear(uint32_t color) {
	uint16_t clear_color_disp = to_disp_color(color);

	uint16_t cs = 0;
	uint16_t ce = display_width - 1;
	uint16_t ps = 0;
	uint16_t pe = display_height - 1;

	uint8_t col[2] = {cs, ce};
	uint8_t row[2] = {ps, pe};

	disp_ssd1351_command(0x15, col, 2);
	disp_ssd1351_command(0x75, row, 2);

	hwspi_begin();
	command_start(0x5C);
	hwspi_data_stream_start();
	for (int i = 0; i < (display_width * display_height); i ++) {
		hwspi_data_stream_write((uint8_t)(clear_color_disp));
		hwspi_data_stream_write((uint8_t)(clear_color_disp >> 8));
	}
	hwspi_data_stream_finish();
	hwspi_end();
}

static lbm_value ext_disp_cmd(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	lbm_value res = ENC_SYM_TERROR;

	if (argn > 1) {
		uint8_t cmd = (uint8_t)lbm_dec_as_u32(args[0]);
		uint8_t paras[12];
		for (int i = 0; i < argn - 1; i ++) {
			paras[i] = (uint8_t)lbm_dec_as_u32(args[i + 1]);
		}

		disp_ssd1351_command(cmd, paras, argn - 1);

		res = ENC_SYM_TRUE;
	} else if (argn == 1) {
		uint8_t cmd = (uint8_t) lbm_dec_as_u32(args[0]);
		disp_ssd1351_command(cmd, 0, 0);
		res = ENC_SYM_TRUE;
	}

	return res;
}

void disp_ssd1351_init(int pin_sd0, int pin_clk, int pin_cs, int pin_reset, int pin_dc, int clock_mhz) {
	hwspi_init(clock_mhz, 0, -1, pin_sd0, pin_clk, pin_cs);
	m_pin_reset = pin_reset;
	m_pin_dc    = pin_dc;

	gpio_config_t gpconf = {0};
	gpconf.pin_bit_mask = BIT(m_pin_reset) | BIT(m_pin_dc);
	gpconf.mode = GPIO_MODE_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpconf.intr_type =  GPIO_INTR_DISABLE;

	gpio_config(&gpconf);

	gpio_set_level(m_pin_reset, 1);
	gpio_set_level(m_pin_dc, 0);

	lbm_add_extension("ext-disp-cmd", ext_disp_cmd);
}


void disp_ssd1351_command(uint8_t command, const uint8_t *args, int argn) {
	hwspi_begin();
	command_start(command);
	if (args != NULL && argn > 0) {
		hwspi_send_data(args, argn);
	}
	hwspi_end();
}

static const uint8_t init_cmds[][5] = {
		{2, 0xFD, 0xB1},             // SSD1351_CMD_COMMANDLOCK
		{1, 0xAE},                   // SSD1351_CMD_DISPLAYOFF
		{3, 0x15, 0x00, 0x7F},       // SSD1351_CMD_SETCOLUMN
		{3, 0x75, 0x00, 0x7F},       // SSD1351_CMD_SETROW
		{2, 0xA0, 0x74},             // SSD1351_CMD_SETREMAP
		{2, 0xA1, 0x00},             // SSD1351_CMD_STARTLINE
		{2, 0xA2, 0x00},             // SSD1351_CMD_DISPLAYOFFSET
		{1, 0xA6},                   // SSD1351_CMD_NORMALDISPLAY
		{2, 0xAB, 0x01},             // SSD1351_CMD_FUNCTIONSELECT
		{1, 0xAF},                   // SSD1351_CMD_DISPLAYON
		{2, 0xB1, 0x32},             // SSD1351_CMD_PRECHARGE
		{2, 0xB3, 0xF1},             // SSD1351_CMD_CLOCKDIV
		{4, 0xB4, 0xA0, 0xB5, 0x55}, // SSD1351_CMD_SETVSL
		{2, 0xB5, 0xA0},             // SSD1351_CMD_SETGPIO
		{2, 0xB6, 0x01},             // SSD1351_CMD_PRECHARGE2
		{2, 0xBB, 0x17},             // SSD1351_CMD_PRECHARGELEVEL
		{2, 0xBE, 0x05},             // SSD1351_CMD_VCOMH
		{4, 0xC1, 0x88, 0x70, 0x88}, // SSD1351_CMD_CONTRASTABC
		{2, 0xC7, 0x0F},             // SSD1351_CMD_CONTRASTMASTER
		{2, 0xCA, 0x7F},             // SSD1351_CMD_MUXRATIO
};

//static const uint8_t gray_table[] = {
//		0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
//		14, 15, 16, 17, 18, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 42, 45,
//		48, 51, 54, 57, 60, 63, 66, 69, 72, 76, 80, 84, 88, 92, 96, 100, 104,
//		108, 112, 116, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170,
//		175, 180
//};

void disp_ssd1351_reset(void) {
	gpio_set_level(m_pin_reset, 0);
	vTaskDelay(500);
	gpio_set_level(m_pin_reset, 1);
	vTaskDelay(120);

	for (int i = 0; i < 20; i ++) {
		int argn = init_cmds[i][0] - 1;
		const uint8_t *args = &init_cmds[i][2];
		uint8_t  cmd  = init_cmds[i][1];
		disp_ssd1351_command(cmd, args, argn);
	}

	// Gray Table
//	disp_ssd1351_command(0xB8, gray_table, sizeof(gray_table));
	disp_ssd1351_command(0xB9, 0, 0);

	// Display on
	disp_ssd1351_command(0xAF, 0, 0);

	disp_ssd1351_clear(0);
}
