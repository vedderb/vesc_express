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

#include "disp_ili9488.h"
#include "hwspi.h"
#include "lispif.h"
#include "lispbm.h"


static int display_width;
static int display_height;

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

static void blast_indexed2(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 3;
		int bit  = 7 - (i & 0x7);
		int color_ind = (data[byte] & (1 << bit)) >> bit;

		uint32_t c = COLOR_TO_RGB888(colors[color_ind], i % img->width, i / img->width);
		hwspi_data_stream_write((uint8_t)(c >> 16));
		hwspi_data_stream_write((uint8_t)(c >> 8));
		hwspi_data_stream_write((uint8_t)(c >> 0));
	}

	hwspi_data_stream_finish();
}

static void blast_indexed4(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 2;
		int bit = (3 - (i & 0x03)) * 2;
		int color_ind = (data[byte] & (0x03 << bit)) >> bit;

		uint32_t c = COLOR_TO_RGB888(colors[color_ind], i % img->width, i / img->width);
		hwspi_data_stream_write((uint8_t)(c >> 16));
		hwspi_data_stream_write((uint8_t)(c >> 8));
		hwspi_data_stream_write((uint8_t)(c >> 0));
	}

	hwspi_data_stream_finish();
}

static void blast_indexed16(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 1;    // byte to access is pix / 2
		int bit = (1 - (i & 0x01)) * 4; // bit position to access within byte
		int color_ind = (data[byte] & (0x0F << bit)) >> bit; // extract 4 bit value.

		uint16_t c = COLOR_TO_RGB888(colors[color_ind],	i % img->width, i / img->width);
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}


static void blast_rgb332(uint8_t *data, uint32_t num_pix) {
	command_start(0x2C);
	hwspi_data_stream_start();

	for (int i = 0; i < num_pix; i ++) {
		uint8_t pix = data[i];
		uint32_t r = (uint32_t)((pix >> 5) & 0x7);
		uint32_t g = (uint32_t)((pix >> 2) & 0x7);
		uint32_t b = (uint32_t)(pix & 0x3);
		uint32_t c = r << (16 + 5) | g << (8 + 5) | b << 6;
		hwspi_data_stream_write((uint8_t)(c >> 16));
		hwspi_data_stream_write((uint8_t)(c >> 8));
		hwspi_data_stream_write((uint8_t)(c >> 0));
	}

	hwspi_data_stream_finish();
}

static void blast_rgb565(uint8_t *data, uint32_t num_pix) {
	command_start(0x2C);
	hwspi_data_stream_start();

	for (int i = 0; i < num_pix; i ++) {
		uint16_t pix = (((uint16_t)data[2 * i]) << 8) | ((uint16_t)data[2 * i + 1]);

		uint32_t r = (uint32_t)(pix >> 11);
		uint32_t g = (uint32_t)((pix >> 5) & 0x3F);
		uint32_t b = (uint32_t)(pix & 0x1F);
		uint32_t c = r << (16 + 3) | g << (8 + 2) | b << 3;
		hwspi_data_stream_write((uint8_t)(c >> 16));
		hwspi_data_stream_write((uint8_t)(c >> 8));
		hwspi_data_stream_write((uint8_t)(c >> 0));
	}

	hwspi_data_stream_finish();
}

static void blast_rgb888(uint8_t *data, uint32_t num_pix) {
	command_start(0x2C);
	hwspi_data_stream_start();

	for (int i = 0; i < num_pix; i ++) {
		hwspi_data_stream_write(data[3 * i]);
		hwspi_data_stream_write(data[3 * i + 1]);
		hwspi_data_stream_write(data[3 * i + 2]);
	}

	hwspi_data_stream_finish();
}

bool disp_ili9488_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) {
	uint16_t cs = x;
	uint16_t ce = x + img->width - 1;
	uint16_t ps = y;
	uint16_t pe = y + img->height - 1;

	if (ce >= display_width || pe >= display_height) {
		return false;
	}

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

	disp_ili9488_command(0x2A, col, 4);
	disp_ili9488_command(0x2B, row, 4);

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

void disp_ili9488_clear(uint32_t color) {
	uint16_t cs = 0;
	uint16_t ce = display_width - 1;
	uint16_t ps = 0;
	uint16_t pe = display_height - 1;

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

	disp_ili9488_command(0x2A, col, 4);
	disp_ili9488_command(0x2B, row, 4);

	hwspi_begin();
	command_start(0x2C);
	hwspi_data_stream_start();
	for (int i = 0; i < (display_width * display_height); i ++) {
		hwspi_data_stream_write((uint8_t)(color >> 16));
		hwspi_data_stream_write((uint8_t)(color >> 8));
		hwspi_data_stream_write((uint8_t)(color >> 0));
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

		disp_ili9488_command(cmd, paras, argn - 1);

		res = ENC_SYM_TRUE;
	} else if (argn == 1) {
		uint8_t cmd = (uint8_t) lbm_dec_as_u32(args[0]);
		disp_ili9488_command(cmd, 0, 0);
		res = ENC_SYM_TRUE;
	}

	return res;
}

static lbm_value ext_disp_orientation(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint32_t orientation = lbm_dec_as_u32(args[0]);
	uint8_t arg = 0;
	lbm_value res = ENC_SYM_TRUE;
	switch(orientation) {
	case 0:
		arg = 0x48;
		disp_ili9488_command(0x36, &arg, 1);
		display_width = 320;
		display_height = 480;
		break;
	case 1:
		arg = 0x28;
		disp_ili9488_command(0x36, &arg, 1);
		display_width = 480;
		display_height = 320;
		break;
	case 2:
		arg = 0x98;
		disp_ili9488_command(0x36, &arg, 1);
		display_width = 320;
		display_height = 480;
		break;
	case 3:
		arg = 0xE8;
		disp_ili9488_command(0x36, &arg, 1);
		display_width = 480;
		display_height = 320;
		break;
	default:
		res = ENC_SYM_EERROR;
		break;
	}
	return res;
}

void disp_ili9488_init(int pin_sd0, int pin_clk, int pin_cs, int pin_reset, int pin_dc, int clock_mhz) {
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
	lbm_add_extension("ext-disp-orientation", ext_disp_orientation);
}

void disp_ili9488_command(uint8_t command, const uint8_t *args, int argn) {
	hwspi_begin();
	command_start(command);
	if (args != NULL && argn > 0) {
		hwspi_send_data(args, argn);
	}
	hwspi_end();
}

static const uint8_t ili9488_init_sequence[][17] = {
		{16, 0xE0, 0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F},
		{16, 0xE1, 0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F},
		{3, 0xC0, 0x17, 0x15},
		{2, 0xC1, 0x41},
		{4, 0xC5, 0x00, 0x12, 0x80},
		{2, 0x36, 0x48},
		{2, 0x3A, 0x66}, // NOTE: Only the 18-bit mode works, which actually is the 24-bit mode
		{2, 0xB0, 0x00},
		{2, 0xB1, 0xA0},
		{2, 0xB4, 0x02},
		{3, 0xB6, 0x05, 0x02},
		{2, 0xE9, 0x00},
		{5, 0xF7, 0xA9, 0x51, 0x2C, 0x82},
};

void disp_ili9488_reset(void) {
	gpio_set_level(m_pin_reset, 0);
	vTaskDelay(5);
	gpio_set_level(m_pin_reset, 1);
	vTaskDelay(120);

	for (int i = 0; i < 13; i ++) {
		int argn = ili9488_init_sequence[i][0] - 1;
		const uint8_t *args = &ili9488_init_sequence[i][2];
		uint8_t cmd  = ili9488_init_sequence[i][1];
		disp_ili9488_command(cmd, args, argn);
	}
	disp_ili9488_command(0x11, 0, 0);
	vTaskDelay(100);
	disp_ili9488_command(0x29, 0, 0);
	vTaskDelay(100);

	display_width = 320;
	display_height = 480;

	disp_ili9488_clear(0);
}
