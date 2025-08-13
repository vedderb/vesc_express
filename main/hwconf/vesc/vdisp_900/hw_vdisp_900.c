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

#include "hw_vdisp_900.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "extensions.h"
#include "heap.h"
#include "soc/gpio_struct.h"
#include "driver/dedic_gpio.h"
#include "hal/dedic_gpio_cpu_ll.h"
#include "lispif.h"
#include "lispif_disp_extensions.h"

#define GPIO_EXP_IN0_REG 0x0
#define GPIO_EXP_IN1_REG 0x1

#define GPIO_EXP_OUT0_REG 0x2
#define GPIO_EXP_OUT1_REG 0x3

#define GPIO_EXP_CONF0_REG 0x6
#define GPIO_EXP_CONF1_REG 0x7

#define GPIO_EXP_UPDATE_PORT0()                                                \
	i2c_write_reg(I2C_ADDR_TCA9535, GPIO_EXP_OUT0_REG, io_port0)

#define DISP_REG_SET GPIO.out_w1ts.val
#define DISP_REG_CLR GPIO.out_w1tc.val

#define WR0() (DISP_REG_CLR = 1 << DISP_WR)
#define WR1() (DISP_REG_SET = 1 << DISP_WR)
#define DISP_PORT_SET(data)                                                    \
	DISP_REG_CLR = 0x4FF;                                                      \
	DISP_REG_SET = ((data)&0xFF)

#define CLEAR_CS()                                                             \
	io_port0 |= (1 << EXP_DISP_DC);                                            \
	GPIO_EXP_UPDATE_PORT0()

#define SET_CS()                                                               \
	io_port0 &= ~(1 << EXP_DISP_DC);                                           \
	GPIO_EXP_UPDATE_PORT0()

static SemaphoreHandle_t i2c_mutex;
static uint8_t io_port0 = 0;

static int display_width  = 0;
static int display_height = 0;

static int io_array[]                       = {10, 0, 2, 3, 4, 5, 6, 7};
static dedic_gpio_bundle_config_t io_config = {
	.gpio_array = io_array,
	.array_size = 8,
	.flags      = {.out_en = 1, .in_en = 0, .out_invert = 0, .in_invert = 0}};
static dedic_gpio_bundle_handle_t io_handle;

static esp_err_t i2c_tx_rx(
	uint8_t addr, const uint8_t *write_buffer, size_t write_size,
	uint8_t *read_buffer, size_t read_size
) {

	xSemaphoreTake(i2c_mutex, portMAX_DELAY);

	esp_err_t res;
	if (read_size > 0 && read_buffer != NULL) {
		if (write_size > 0 && write_buffer != NULL) {
			res = i2c_master_write_read_device(
				0, addr, write_buffer, write_size, read_buffer, read_size, 2000
			);
		} else {
			res = i2c_master_read_from_device(
				0, addr, read_buffer, read_size, 2000
			);
		}
	} else {
		res =
			i2c_master_write_to_device(0, addr, write_buffer, write_size, 2000);
	}
	xSemaphoreGive(i2c_mutex);

	return res;
}

static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
	uint8_t tx_buf[2] = {reg, val};
	return i2c_tx_rx(addr, tx_buf, 2, 0, 0);
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

static lbm_value ext_disp_set_bl(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int val = lbm_dec_as_u32(args[0]);

	io_port0 &= 0b00011111;

	switch (val) {
		case 0:
			io_port0 |= 0b00000000;
			break;

		case 1:
			io_port0 |= 0b10000000;
			break;

		case 2:
			io_port0 |= 0b01000000;
			break;

		case 3:
			io_port0 |= 0b11000000;
			break;

		case 4:
			io_port0 |= 0b00100000;
			break;

		case 5:
			io_port0 |= 0b10100000;
			break;

		case 6:
			io_port0 |= 0b01100000;
			break;

		case 7:
			io_port0 |= 0b11100000;
			break;

		default:
			break;
	}

	GPIO_EXP_UPDATE_PORT0();

	return ENC_SYM_TRUE;
}

static void io_restore(void) {
	dedic_gpio_del_bundle(io_handle);

	gpio_config_t gpconf = {0};
	gpconf.pin_bit_mask  = 0xff | BIT(DISP_WR);
	gpconf.mode          = GPIO_MODE_OUTPUT;
	gpconf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en    = GPIO_PULLUP_DISABLE;
	gpconf.intr_type     = GPIO_INTR_DISABLE;

	gpio_config(&gpconf);
}

static void disp_command_start(uint8_t cmd) {
	io_port0 &= 0b111110001;
	io_port0 |= 0b000000011;
	WR1();
	GPIO_EXP_UPDATE_PORT0();

	WR0();
	DISP_PORT_SET(cmd);
	WR1();

	io_port0 |= (1 << EXP_DISP_DC);
	GPIO_EXP_UPDATE_PORT0();
}

static void disp_command(uint8_t command, const uint8_t *args, int argn) {
	disp_command_start(command);

	for (int i = 0; i < argn; i++) {
		WR0();
		DISP_PORT_SET(args[i]);
		WR1();
	}

	SET_CS();
}

static void blast_indexed2(image_buffer_t *img, color_t *colors) {
	disp_command_start(0x2C);

	dedic_gpio_new_bundle(&io_config, &io_handle);

	uint8_t *data = img->data;
	int num_pix   = img->width * img->height;

	uint32_t bus_data = 1;
	dedic_gpio_cpu_ll_write_all(bus_data);

	for (int i = 0; i < num_pix; i++) {
		int byte      = i >> 3;
		int bit       = 7 - (i & 0x7);
		int color_ind = (data[byte] & (1 << bit)) >> bit;

		uint32_t color =
			COLOR_TO_RGB888(colors[color_ind], i % img->width, i / img->width);

		bus_data = (color >> 16) & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		bus_data = (color >> 8) & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		bus_data = color & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);
	}

	io_restore();
}

static void blast_indexed4(image_buffer_t *img, color_t *colors) {
	disp_command_start(0x2C);

	dedic_gpio_new_bundle(&io_config, &io_handle);

	uint8_t *data = img->data;
	int num_pix   = img->width * img->height;

	uint32_t bus_data = 1;
	dedic_gpio_cpu_ll_write_all(bus_data);

	for (int i = 0; i < num_pix; i++) {
		int byte      = i >> 2;
		int bit       = (3 - (i & 0x03)) * 2;
		int color_ind = (data[byte] & (0x03 << bit)) >> bit;

		uint32_t color =
			COLOR_TO_RGB888(colors[color_ind], i % img->width, i / img->width);

		bus_data = (color >> 16) & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		bus_data = (color >> 8) & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		bus_data = color & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);
	}

	io_restore();
}

static void blast_indexed16(image_buffer_t *img, color_t *colors) {
	disp_command_start(0x2C);

	dedic_gpio_new_bundle(&io_config, &io_handle);

	uint8_t *data = img->data;
	int num_pix   = img->width * img->height;

	uint32_t bus_data = 1;
	dedic_gpio_cpu_ll_write_all(bus_data);

	for (int i = 0; i < num_pix; i++) {
		int byte = i >> 1;               // byte to access is pix / 2
		int bit  = (1 - (i & 0x01)) * 4; // bit position to access within byte
		int color_ind = (data[byte] & (0x0F << bit))
			>> bit; // extract 4 bit value.

		uint32_t color =
			COLOR_TO_RGB888(colors[color_ind], i % img->width, i / img->width);

		bus_data = (color >> 16) & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		bus_data = (color >> 8) & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		bus_data = color & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);
	}

	io_restore();
}

static void blast_rgb332(uint8_t *data, uint32_t num_pix) {
	disp_command_start(0x2C);

	dedic_gpio_new_bundle(&io_config, &io_handle);

	uint32_t bus_data = 1;
	dedic_gpio_cpu_ll_write_all(bus_data);

	for (int i = 0; i < num_pix; i++) {
		uint8_t pix = data[i];
		uint32_t r  = (uint32_t)((pix >> 5) & 0x7);
		uint32_t g  = (uint32_t)((pix >> 2) & 0x7);
		uint32_t b  = (uint32_t)(pix & 0x3);

		bus_data = r & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		bus_data = g & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		bus_data = b & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);
	}

	io_restore();
}

static void blast_rgb565(uint8_t *data, uint32_t num_pix) {
	disp_command_start(0x2C);

	dedic_gpio_new_bundle(&io_config, &io_handle);

	uint32_t bus_data = 1;
	dedic_gpio_cpu_ll_write_all(bus_data);

	for (int i = 0; i < num_pix; i++) {
		uint16_t pix = (((uint16_t)data[2 * i]) << 8)
			| ((uint16_t)data[2 * i + 1]);

		uint32_t r = (uint32_t)(pix >> 11);
		uint32_t g = (uint32_t)((pix >> 5) & 0x3F);
		uint32_t b = (uint32_t)(pix & 0x1F);

		bus_data = r & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		bus_data = g & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		bus_data = b & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);
	}

	io_restore();
}

static void blast_rgb888(uint8_t *data, uint32_t num_pix) {
	disp_command_start(0x2C);

	dedic_gpio_new_bundle(&io_config, &io_handle);

	uint32_t bus_data = 1;
	dedic_gpio_cpu_ll_write_all(bus_data);

	for (int i = 0; i < num_pix; i++) {
		uint32_t r = data[3 * i];
		uint32_t g = data[3 * i + 1];
		uint32_t b = data[3 * i + 2];

		bus_data = r & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		bus_data = g & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		bus_data = b & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);
		
		//DISP_PORT_SET(r);
		//WR1();
		//DISP_PORT_SET(g);
		//WR1();
		//DISP_PORT_SET(b);
		//WR1();
	}

	io_restore();
}

bool disp_render_image(
	image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors
) {
	uint16_t cs = x;
	uint16_t ce = x + img->width - 1;
	uint16_t ps = y;
	uint16_t pe = y + img->height - 1;

	if (ce >= display_width || pe >= display_height) {
		return false;
	}

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

	disp_command(0x2A, col, 4);
	disp_command(0x2B, row, 4);

	uint32_t num_pix = img->width * img->height;

	switch (img->fmt) {
		case indexed2:
			if (!colors)
				return false;
			blast_indexed2(img, colors);
			break;
		case indexed4:
			if (!colors)
				return false;
			blast_indexed4(img, colors);
			break;
		case indexed16:
			if (!colors)
				return false;
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

	SET_CS();

	return true;
}

static void disp_clear(uint32_t color) {
	uint16_t cs = 0;
	uint16_t ce = display_width - 1;
	uint16_t ps = 0;
	uint16_t pe = display_height - 1;

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

	disp_command(0x2A, col, 4);
	disp_command(0x2B, row, 4);

	disp_command_start(0x2C);

	dedic_gpio_new_bundle(&io_config, &io_handle);

	uint32_t bus_data = 1;
	dedic_gpio_cpu_ll_write_all(bus_data);

	for (int i = 0; i < (display_width * display_height); i++) {
		bus_data = (color >> 16) & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		bus_data = (color >> 8) & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		bus_data = color & 0xFC;
		dedic_gpio_cpu_ll_write_all(bus_data);
		dedic_gpio_cpu_ll_write_all(bus_data);
		bus_data |= 0b00000001;
		dedic_gpio_cpu_ll_write_all(bus_data);

		//DISP_PORT_SET(color >> 16);
		//WR1();
		//DISP_PORT_SET(color >> 8);
		//WR1();
		//DISP_PORT_SET(color >> 0);
		//WR1();
	}

	io_restore();
	SET_CS();
}

static const uint8_t init_cmds[][16] = {
	{2, 0xf0, 0xc3},
	{2, 0xf0, 0x96},
	{2, 0x36, 0b00101000},
	{2, 0xb4, 0x01},
	{2, 0xb7, 0xc6},

	{9, 0xe8, 0x40, 0x8a, 0x00, 0x00, 0x29, 0x19, 0xa5, 0x33},

	{2, 0xc1, 0x00},
	{2, 0xc2, 0xa7},
	{2, 0xc5, 0x08},

	{15, 0xe0, 0xf0, 0x06, 0x0b, 0x07, 0x06, 0x05, 0x2e, 0x33, 0x47, 0x3a, 0x17,
	 0x16, 0x2e, 0x31},

	{15, 0xe1, 0xf0, 0x09, 0x0d, 0x09, 0x08, 0x23, 0x2e, 0x33, 0x46, 0x38, 0x13,
	 0x13, 0x2c, 0x32},

	{2, 0xf0, 0x3c},
	{2, 0xf0, 0x69},
	{2, 0x3a, 0x06},
};

void disp_reset(void) {
	gpio_config_t gpconf = {0};
	gpconf.pin_bit_mask  = 0xff | BIT(DISP_WR);
	gpconf.mode          = GPIO_MODE_OUTPUT;
	gpconf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en    = GPIO_PULLUP_DISABLE;
	gpconf.intr_type     = GPIO_INTR_DISABLE;

	gpio_config(&gpconf);

	io_port0 &= ~BIT(EXP_DISP_RESET);
	GPIO_EXP_UPDATE_PORT0();
	vTaskDelay(100);
	io_port0 |= BIT(EXP_DISP_RESET);
	GPIO_EXP_UPDATE_PORT0();
	vTaskDelay(220);

	disp_command(0x11, 0, 0);
	vTaskDelay(220);

	for (int i = 0; i < 14; i++) {
		int argn            = init_cmds[i][0] - 1;
		const uint8_t *args = &init_cmds[i][2];
		uint8_t cmd         = init_cmds[i][1];
		disp_command(cmd, args, argn);
	}

	disp_command(0x21, 0, 0);

	vTaskDelay(120);
	disp_command(0x29, 0, 0);
	
	display_width  = 480;
	display_height = 320;
}

static lbm_value ext_disp_cmd(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	lbm_value res = ENC_SYM_TERROR;

	if (argn > 1) {
		uint8_t cmd = (uint8_t)lbm_dec_as_u32(args[0]);
		uint8_t paras[12];
		for (int i = 0; i < argn - 1; i++) {
			paras[i] = (uint8_t)lbm_dec_as_u32(args[i + 1]);
		}

		disp_command(cmd, paras, argn - 1);

		res = ENC_SYM_TRUE;
	} else if (argn == 1) {
		uint8_t cmd = (uint8_t)lbm_dec_as_u32(args[0]);
		disp_command(cmd, 0, 0);
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
		arg = 0b1001000;
		disp_command(0x36, &arg, 1);
		display_width = 320;
		display_height = 480;
		break;
	case 1:
		arg = 0b11101000;
		disp_command(0x36, &arg, 1);
		display_width = 480;
		display_height = 320;
		break;
	case 2:
		arg = 0b10001000;
		disp_command(0x36, &arg, 1);
		display_width = 320;
		display_height = 480;
		break;
	case 3:
		arg = 0b00101000;
		disp_command(0x36, &arg, 1);
		display_width = 480;
		display_height = 320;
		break;
	default:
		res = ENC_SYM_TERROR;
		break;
	}
	return res;
}

static lbm_value ext_btn_pull_en(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);
	
	i2c_write_reg(I2C_ADDR_TCA9535, GPIO_EXP_OUT1_REG, BIT(EXP_P2_BTN1_PULL) | BIT(EXP_P2_BTN2_PULL));
	
	if (lbm_is_symbol_nil(args[0])) {
		i2c_write_reg(I2C_ADDR_TCA9535, GPIO_EXP_CONF1_REG, 0xFF);
	} else {
		i2c_write_reg(I2C_ADDR_TCA9535, GPIO_EXP_CONF1_REG, (~(BIT(EXP_P2_BTN1_PULL) | BIT(EXP_P2_BTN2_PULL))) & 0xFF);
	}

	return ENC_SYM_TRUE;
}

static void load_extensions(bool main_found) {
	// IO Expander
	i2c_write_reg(I2C_ADDR_TCA9535, GPIO_EXP_CONF0_REG, 0x10);
	i2c_write_reg(I2C_ADDR_TCA9535, GPIO_EXP_OUT0_REG, 0b00001111);

	lbm_display_extensions_set_callbacks(
		disp_render_image, disp_clear, disp_reset
	);

	if (main_found) {
		return;
	}

	lbm_add_extension("disp-set-bl", ext_disp_set_bl);
	lbm_add_extension("disp-cmd", ext_disp_cmd);
	lbm_add_extension("disp-orientation", ext_disp_orientation);
	lbm_add_extension("btn-pull-en", ext_btn_pull_en);

	// Replace existing I2C-extensions
	lbm_add_extension("i2c-start", ext_i2c_start);
	lbm_add_extension("i2c-tx-rx", ext_i2c_tx_rx);
	lbm_add_extension("i2c-detect-addr", ext_i2c_detect_addr);
}

void hw_init(void) {
	i2c_mutex = xSemaphoreCreateMutex();

	i2c_config_t conf = {
		.mode             = I2C_MODE_MASTER,
		.sda_io_num       = I2C_SDA,
		.scl_io_num       = I2C_SCL,
		.sda_pullup_en    = GPIO_PULLUP_ENABLE,
		.scl_pullup_en    = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 400000,
	};
	i2c_param_config(0, &conf);
	i2c_driver_install(0, conf.mode, 0, 0, 0);

	gpio_config_t gpconf = {0};
	gpconf.pin_bit_mask  = 0xff | BIT(DISP_WR);
	gpconf.mode          = GPIO_MODE_OUTPUT;
	gpconf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en    = GPIO_PULLUP_DISABLE;
	gpconf.intr_type     = GPIO_INTR_DISABLE;

	gpio_config(&gpconf);

	lispif_add_ext_load_callback(load_extensions);
}
