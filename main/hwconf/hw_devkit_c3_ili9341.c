/*
	Copyright 2023 Joel Svensson	svenssonjoel@yahoo.se
	          2023 Benjamin Vedder  benjamin@vedder.se

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

#include "hw_devkit_c3.h"
#include "lispif.h"
#include "lispbm.h"
#include "lispif_disp_extensions.h"
#include "commands.h"

#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#define GPIO_DISP_RESET        18
#define GPIO_DISP_SPI_MISO     4
#define GPIO_DISP_SPI_CLK      5
#define GPIO_DISP_SPI_MOSI     6
#define GPIO_DISP_DATA_COMMAND 7
#define GPIO_DISP_SPI_CS       19

#define DISP_REG_SET		GPIO.out_w1ts.val
#define DISP_REG_CLR		GPIO.out_w1tc.val

#define SET_RESET() 			(DISP_REG_SET = 1 << GPIO_DISP_RESET)
#define CLEAR_RESET() 		 	(DISP_REG_CLR = 1 << GPIO_DISP_RESET)
#define SET_DATA_COMMAND() 	    (DISP_REG_SET = 1 << GPIO_DISP_DATA_COMMAND)
#define CLEAR_DATA_COMMAND() 	(DISP_REG_CLR = 1 << GPIO_DISP_DATA_COMMAND)

#define TRANS_1 GPIO_NUM_8
#define TRANS_2 GPIO_NUM_1
#define BUF_SWITCH 10

void init_gpio(void) {
	gpio_reset_pin(GPIO_DISP_RESET);
	gpio_reset_pin(GPIO_DISP_SPI_MOSI);
	gpio_reset_pin(GPIO_DISP_DATA_COMMAND);
	gpio_reset_pin(GPIO_DISP_SPI_CLK);
	gpio_reset_pin(GPIO_DISP_SPI_CS);
	
	gpio_config_t gpconf = {0};

	gpconf.pin_bit_mask =
			BIT(GPIO_DISP_RESET) |
			BIT(GPIO_DISP_DATA_COMMAND) |
			BIT(TRANS_1) |
			BIT(TRANS_2) |
			BIT(BUF_SWITCH) |
			0;
	gpconf.mode = GPIO_MODE_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_ENABLE;
	gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpconf.intr_type =  0;
	gpio_config(&gpconf);

	CLEAR_DATA_COMMAND();
	gpio_set_level(TRANS_1,0);
	gpio_set_level(TRANS_2,0);
	gpio_set_level(BUF_SWITCH,0);

}

static void disp_spi_tx_pre(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(GPIO_DISP_DATA_COMMAND, dc);
}

static spi_device_handle_t spi;

void init_hwspi(void) {
	spi_bus_config_t buscfg={
			.miso_io_num=GPIO_DISP_SPI_MISO,
			.mosi_io_num=GPIO_DISP_SPI_MOSI,
			.sclk_io_num=GPIO_DISP_SPI_CLK,
			.quadwp_io_num=-1,
			.quadhd_io_num=-1,
			.max_transfer_sz=16*320*2+8
	};
	spi_device_interface_config_t devcfg={
			.clock_speed_hz=40*1000*1000,
			.mode=0,                                
			.spics_io_num=GPIO_DISP_SPI_CS,        
			.flags = 0,
			.queue_size=7,                          
			.pre_cb=disp_spi_tx_pre,               
	};
	spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
	spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
}

void disp_write_cmd(uint8_t cmd) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=8;
    t.tx_buffer=&cmd;
    t.user=(void*)0;
    spi_device_polling_transmit(spi, &t);
}

void disp_write_data(uint8_t *data, int len) {
	spi_transaction_t t;
	if (len==0) return;        
	memset(&t, 0, sizeof(t)); 
	t.length=len*8;            
	t.tx_buffer=data;  
	t.user=(void*)1;              
	spi_device_polling_transmit(spi, &t);  //Transmit!
}

static lbm_value ext_disp_reset(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	CLEAR_RESET();
	vTaskDelay(200);
	SET_RESET();
	return ENC_SYM_TRUE;
}

static uint16_t to_RGB565(uint32_t rgb) {
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

static lbm_value ext_disp_rect(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	if (argn == 5) {

		uint16_t x = (uint16_t)lbm_dec_as_u32(args[0]);
		uint16_t y = (uint16_t)lbm_dec_as_u32(args[1]);
		uint16_t w = (uint16_t)lbm_dec_as_u32(args[2]);
		uint16_t h = (uint16_t)lbm_dec_as_u32(args[3]);
		uint16_t x1 = w + x;
		uint16_t y1 = h + y;
		uint16_t c = to_RGB565(lbm_dec_as_u32(args[4]));
		
		spi_device_acquire_bus(spi, portMAX_DELAY);
		uint8_t data[4];
		data[0] = x>>8; data[1] = x;
		data[2] = x1>>8; data[3] = x1;
		disp_write_cmd(0x2A); // Col addr
		disp_write_data(data, 4);
		data[0] = y>>8; data[1] = y;
		data[2] = y1>>8; data[3] = y1;
		disp_write_cmd(0x2B); // page addr
		disp_write_data(data,4);

		uint16_t colors[16];
		int image_size = w*h;
		int blocks = image_size >> 4;
		int rest   = image_size & 0xF;
		
		for ( int j = 0; j < 16; j ++) {
			colors[j] = c;
		}

		disp_write_cmd(0x2C);
		for (int i = 0; i < blocks; i ++) {
			disp_write_data((uint8_t*)colors, 32);
		}
		
		disp_write_data((uint8_t*)colors, rest*2);
		
		spi_device_release_bus(spi);
		return ENC_SYM_TRUE;
	}
	return ENC_SYM_TERROR;
}

static lbm_value ext_disp_cmd(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	uint8_t cmd_args[10];
	int     cmd_argn = 0;

	if (argn >= 1 && argn < 10){

		for (int i = 1; i < argn; i++) {
			cmd_args[i-1] = (uint8_t)lbm_dec_as_u32(args[i]);
			cmd_argn++;
		}

		spi_device_acquire_bus(spi, portMAX_DELAY);
		disp_write_cmd((uint8_t)lbm_dec_as_u32(args[0]));
		disp_write_data(cmd_args, cmd_argn);
		spi_device_release_bus(spi);
		return ENC_SYM_TRUE;
	}
	return ENC_SYM_TERROR;
}

#define DISP_DATA_BUFFER_SIZE 1024

typedef struct data_stream_buffer_s {
	uint8_t data[DISP_DATA_BUFFER_SIZE];
	int     pos;
	spi_transaction_t trans;
	struct data_stream_buffer_s *next;
} data_stream_buffer_t;

data_stream_buffer_t data_buffer[2];
data_stream_buffer_t *active_buffer;

static void disp_data_stream_init(void) {
	data_buffer[0].pos = 0;
	data_buffer[0].next = &data_buffer[1];
	data_buffer[0].trans.length = DISP_DATA_BUFFER_SIZE * 8;
	data_buffer[0].trans.tx_buffer = data_buffer[0].data;
	data_buffer[0].trans.flags = SPI_TRANS_CS_KEEP_ACTIVE;
	data_buffer[0].trans.user = (void*)1;
	data_buffer[1].pos = 0;
	data_buffer[1].next = NULL;
	data_buffer[1].trans.length = DISP_DATA_BUFFER_SIZE * 8;
	data_buffer[1].trans.tx_buffer = data_buffer[1].data;
	data_buffer[1].trans.flags = SPI_TRANS_CS_KEEP_ACTIVE;
	data_buffer[1].trans.user = (void*)1;

	active_buffer = &data_buffer[0];
}

static uint32_t buffer0_cnt = 0;
static uint32_t buffer1_cnt = 0;
static uint32_t no_mem_err = 0;
static uint32_t invalid_state_err = 0;
static uint32_t other_err = 0;

static void IRAM_ATTR disp_data_stream_write(uint8_t byte, bool done) {

	if (active_buffer == NULL) {
		while (true) {

			spi_transaction_t   *p;
			esp_err_t r = spi_device_get_trans_result(spi, &p, 0);
			if (r == ESP_OK) {
				if (p == &data_buffer[0].trans) {
					//commands_printf_lisp("buffer 0 done");
					active_buffer = &data_buffer[0];
					active_buffer->next = NULL;
					active_buffer->pos = 0;
					break;
				} else if (p == &data_buffer[1].trans) {
					//commands_printf_lisp("buffer 1 done");
					active_buffer = &data_buffer[1];
					active_buffer->next = NULL;
					active_buffer->pos = 0;
					break;
				} else {
					commands_printf_lisp("This should not happen");
				}
			}
		}
	}

	active_buffer->data[active_buffer->pos++] = byte;
	if (active_buffer->pos == DISP_DATA_BUFFER_SIZE) {
		if (done) {
			active_buffer->trans.flags = 0;
		}

		active_buffer->trans.length = DISP_DATA_BUFFER_SIZE * 8;
		esp_err_t r = spi_device_queue_trans(spi, &active_buffer->trans, portMAX_DELAY);
		if (r == ESP_ERR_NO_MEM) {
			no_mem_err ++;
		} else if (r == ESP_ERR_INVALID_STATE) {
			invalid_state_err ++;
		} else if (r != ESP_OK) {
			other_err ++;
		}
		active_buffer = active_buffer->next;
	} else if (done) {
		active_buffer->trans.flags = 0;
		active_buffer->trans.length = (active_buffer->pos) * 8;
		if (active_buffer == &data_buffer[0]) {
			gpio_set_level(TRANS_1, 1);
		} else {
			gpio_set_level(TRANS_2, 1);
		}
		spi_device_queue_trans(spi, &active_buffer->trans, portMAX_DELAY);
		active_buffer = active_buffer->next;
	}
}


static void IRAM_ATTR blast_indexed2(uint8_t *data, uint32_t *color_map, uint32_t num_pix) {
	uint16_t draw_buf[128];

	int blocks = num_pix >> 7;
	int rest   = num_pix & 0x7F;
	int pos = 0;

	uint16_t colors[2];
	colors[0] = to_RGB565(color_map[0]);
	colors[1] = to_RGB565(color_map[1]);

	spi_device_acquire_bus(spi, portMAX_DELAY);
	disp_data_stream_init();
	disp_write_cmd(0x2C);

	for (int i = 0; i < num_pix -1; i ++) {
		int byte = i >> 3;
		int bit  = 7 - (i & 0x7);
		if (data[byte] & (1 << bit)) {
			disp_data_stream_write((uint8_t)(colors[1]),false);
			disp_data_stream_write((uint8_t)(colors[1] >> 8),false);
		} else {
			disp_data_stream_write((uint8_t)(colors[0]),false);
			disp_data_stream_write((uint8_t)(colors[0] >> 8),false);
		}
	}
	int byte = (num_pix -1) >> 3;
	int bit  = 7 - ((num_pix - 1) & 0x7);
	if (data[byte] & (1 << bit)) {
		disp_data_stream_write((uint8_t)(colors[1]),false);
		disp_data_stream_write((uint8_t)(colors[1] >> 8),true);
	} else {
		disp_data_stream_write((uint8_t)(colors[0]),false);
		disp_data_stream_write((uint8_t)(colors[0] >> 8),true);
	}

	spi_device_release_bus(spi);
}

static void IRAM_ATTR render_image_buffer(image_buffer_t *img, uint32_t *color_map, uint16_t x, uint16_t y) {
	uint16_t cs = x;
	uint16_t ce = x + img->width - 1;
	uint16_t ps = y;
	uint16_t pe = y + img->height - 1;

	spi_device_acquire_bus(spi, portMAX_DELAY);
	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	disp_write_cmd(0x2A);
	disp_write_data(col,4);

	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};
	disp_write_cmd(0x2B);
	disp_write_data(row, 4);
	spi_device_release_bus(spi);

	switch(img->fmt) {
	case indexed2:
		blast_indexed2(img->data + img->data_offset, color_map, img->width * img->height);
		break;
	case indexed4:
		//blast_bpp_2(img, color_map);
		break;
	case rgb332:
		//blast_bpp_8(img, color_map);
		break;
	case rgb565:
		break;
	case rgb888:
		break;
	default:
		break;
	}
}

static lbm_value ext_render(lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_TERROR;
	uint32_t colors[4] = {0};
	if (argn >= 3 &&
		lispif_disp_is_image_buffer(args[0]) &&
		lbm_is_number(args[1]) &&
		lbm_is_number(args[2])) {

		if (argn == 4 &&
			lbm_is_list(args[3])) {
			int i = 0;
			lbm_value curr = args[3];
			while (lbm_is_cons(curr) && i < 4) {
				// Interprete "anything" as a 32bit value
				colors[i] = lbm_dec_as_u32(lbm_car(curr));
				curr = lbm_cdr(curr);
				i++;
			}
		}
		image_buffer_t *img = (image_buffer_t*)lbm_get_custom_value(args[0]);
		render_image_buffer(img, colors, lbm_dec_as_u32(args[1]), lbm_dec_as_u32(args[2]));
		res = ENC_SYM_TRUE;
	}
	return res;
}


static void load_extensions(void) {
	lbm_add_extension("disp-reset", ext_disp_reset);
	lbm_add_extension("disp-rect", ext_disp_rect);
	lbm_add_extension("disp-cmd", ext_disp_cmd);
	lbm_add_extension("disp-render", ext_render);
}

void hw_init(void) {
	init_gpio();
	init_hwspi();
	lispif_set_ext_load_callback(load_extensions);

}
