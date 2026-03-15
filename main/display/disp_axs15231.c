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
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_axs15231b.h"
#include "esp_heap_caps.h"

#include "disp_axs15231.h"
#include "lispif.h"
#include "lispbm.h"
#include <string.h>

#define TAG "AXS15231"

#define DISPLAY_WIDTH_PHYS	320
#define DISPLAY_HEIGHT_PHYS	480

static int m_display_width = DISPLAY_WIDTH_PHYS;
static int m_display_height = DISPLAY_HEIGHT_PHYS;
static int m_rotation = 0;

#define CHUNK_LINES		20
#define PIX_BUF_PIXELS	(DISPLAY_HEIGHT_PHYS * CHUNK_LINES)
#define PIX_BUF_BYTES	(PIX_BUF_PIXELS * 2)

static esp_lcd_panel_io_handle_t m_io    = NULL;
static esp_lcd_panel_handle_t    m_panel = NULL;
static uint8_t *m_pix_buf = NULL;

static const axs15231b_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xBB, (uint8_t []){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5A, 0xA5}, 8, 0},
    {0xA0, (uint8_t []){0xC0, 0x10, 0x00, 0x02, 0x00, 0x00, 0x04, 0x3F, 0x20, 0x05, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00}, 17, 0},
    {0xA2, (uint8_t []){0x30, 0x3C, 0x24, 0x14, 0xD0, 0x20, 0xFF, 0xE0, 0x40, 0x19, 0x80, 0x80, 0x80, 0x20, 0xf9, 0x10, 0x02, 0xff, 0xff, 0xF0, 0x90, 0x01, 0x32, 0xA0, 0x91, 0xE0, 0x20, 0x7F, 0xFF, 0x00, 0x5A}, 31, 0},
    {0xD0, (uint8_t []){0xE0, 0x40, 0x51, 0x24, 0x08, 0x05, 0x10, 0x01, 0x20, 0x15, 0x42, 0xC2, 0x22, 0x22, 0xAA, 0x03, 0x10, 0x12, 0x60, 0x14, 0x1E, 0x51, 0x15, 0x00, 0x8A, 0x20, 0x00, 0x03, 0x3A, 0x12}, 30, 0},
    {0xA3, (uint8_t []){0xA0, 0x06, 0xAa, 0x00, 0x08, 0x02, 0x0A, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x55, 0x55}, 22, 0},
    {0xC1, (uint8_t []){0x31, 0x04, 0x02, 0x02, 0x71, 0x05, 0x24, 0x55, 0x02, 0x00, 0x41, 0x00, 0x53, 0xFF, 0xFF, 0xFF, 0x4F, 0x52, 0x00, 0x4F, 0x52, 0x00, 0x45, 0x3B, 0x0B, 0x02, 0x0d, 0x00, 0xFF, 0x40}, 30, 0},
    {0xC3, (uint8_t []){0x00, 0x00, 0x00, 0x50, 0x03, 0x00, 0x00, 0x00, 0x01, 0x80, 0x01}, 11, 0},
    {0xC4, (uint8_t []){0x00, 0x24, 0x33, 0x80, 0x00, 0xea, 0x64, 0x32, 0xC8, 0x64, 0xC8, 0x32, 0x90, 0x90, 0x11, 0x06, 0xDC, 0xFA, 0x00, 0x00, 0x80, 0xFE, 0x10, 0x10, 0x00, 0x0A, 0x0A, 0x44, 0x50}, 29, 0},
    {0xC5, (uint8_t []){0x18, 0x00, 0x00, 0x03, 0xFE, 0x3A, 0x4A, 0x20, 0x30, 0x10, 0x88, 0xDE, 0x0D, 0x08, 0x0F, 0x0F, 0x01, 0x3A, 0x4A, 0x20, 0x10, 0x10, 0x00}, 23, 0},
    {0xC6, (uint8_t []){0x05, 0x0A, 0x05, 0x0A, 0x00, 0xE0, 0x2E, 0x0B, 0x12, 0x22, 0x12, 0x22, 0x01, 0x03, 0x00, 0x3F, 0x6A, 0x18, 0xC8, 0x22}, 20, 0},
    {0xC7, (uint8_t []){0x50, 0x32, 0x28, 0x00, 0xa2, 0x80, 0x8f, 0x00, 0x80, 0xff, 0x07, 0x11, 0x9c, 0x67, 0xff, 0x24, 0x0c, 0x0d, 0x0e, 0x0f}, 20, 0},
    {0xC9, (uint8_t []){0x33, 0x44, 0x44, 0x01}, 4, 0},
    {0xCF, (uint8_t []){0x2C, 0x1E, 0x88, 0x58, 0x13, 0x18, 0x56, 0x18, 0x1E, 0x68, 0x88, 0x00, 0x65, 0x09, 0x22, 0xC4, 0x0C, 0x77, 0x22, 0x44, 0xAA, 0x55, 0x08, 0x08, 0x12, 0xA0, 0x08}, 27, 0},
    {0xD5, (uint8_t []){0x40, 0x8E, 0x8D, 0x01, 0x35, 0x04, 0x92, 0x74, 0x04, 0x92, 0x74, 0x04, 0x08, 0x6A, 0x04, 0x46, 0x03, 0x03, 0x03, 0x03, 0x82, 0x01, 0x03, 0x00, 0xE0, 0x51, 0xA1, 0x00, 0x00, 0x00}, 30, 0},
    {0xD6, (uint8_t []){0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE, 0x93, 0x00, 0x01, 0x83, 0x07, 0x07, 0x00, 0x07, 0x07, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x84, 0x00, 0x20, 0x01, 0x00}, 30, 0},
    {0xD7, (uint8_t []){0x03, 0x01, 0x0b, 0x09, 0x0f, 0x0d, 0x1E, 0x1F, 0x18, 0x1d, 0x1f, 0x19, 0x40, 0x8E, 0x04, 0x00, 0x20, 0xA0, 0x1F}, 19, 0},
    {0xD8, (uint8_t []){0x02, 0x00, 0x0a, 0x08, 0x0e, 0x0c, 0x1E, 0x1F, 0x18, 0x1d, 0x1f, 0x19}, 12, 0},
    {0xD9, (uint8_t []){0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}, 12, 0},
    {0xDD, (uint8_t []){0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}, 12, 0},
    {0xDF, (uint8_t []){0x44, 0x73, 0x4B, 0x69, 0x00, 0x0A, 0x02, 0x90}, 8,  0},
    {0xE0, (uint8_t []){0x3B, 0x28, 0x10, 0x16, 0x0c, 0x06, 0x11, 0x28, 0x5c, 0x21, 0x0D, 0x35, 0x13, 0x2C, 0x33, 0x28, 0x0D}, 17, 0},
    {0xE1, (uint8_t []){0x37, 0x28, 0x10, 0x16, 0x0b, 0x06, 0x11, 0x28, 0x5C, 0x21, 0x0D, 0x35, 0x14, 0x2C, 0x33, 0x28, 0x0F}, 17, 0},
    {0xE2, (uint8_t []){0x3B, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x35, 0x44, 0x32, 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0D}, 17, 0},
    {0xE3, (uint8_t []){0x37, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x35, 0x44, 0x32, 0x0C, 0x14, 0x14, 0x36, 0x32, 0x2F, 0x0F}, 17, 0},
    {0xE4, (uint8_t []){0x3B, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x39, 0x44, 0x2E, 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0D}, 17, 0},
    {0xE5, (uint8_t []){0x37, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x39, 0x44, 0x2E, 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0F}, 17, 0},
    {0xA4, (uint8_t []){0x85, 0x85, 0x95, 0x82, 0xAF, 0xAA, 0xAA, 0x80, 0x10, 0x30, 0x40, 0x40, 0x20, 0xFF, 0x60, 0x30}, 16, 0},
    {0xA4, (uint8_t []){0x85, 0x85, 0x95, 0x85}, 4, 0},
    {0xBB, (uint8_t []){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8, 0},
    {0x13, (uint8_t []){0x00}, 0, 0},
    {0x11, (uint8_t []){0x00}, 0, 120},
    {0x2C, (uint8_t []){0x00, 0x00, 0x00, 0x00}, 4, 0},
};

static inline uint16_t to_disp_color(uint32_t rgb) {
	uint8_t r = (uint8_t)(rgb >> 16) >> 3;
	uint8_t g = (uint8_t)(rgb >>  8) >> 2;
	uint8_t b = (uint8_t)(rgb)       >> 3;
	uint8_t hi = (r << 3) | (g >> 3);
	uint8_t lo = (g << 5) | b;
	return (uint16_t)hi | ((uint16_t)lo << 8);
}

static inline uint32_t fetch_rgb888(image_buffer_t *img, uint32_t idx, color_t *colors) {
	uint32_t rgb = 0;
	int cur_x = idx % img->width;
	int cur_y = idx / img->width;
	switch (img->fmt) {
	case indexed2:
		if (colors) {
			int ci = (img->data[idx >> 3] >> (7 - (idx & 7))) & 1;
			rgb = COLOR_TO_RGB888(colors[ci], cur_x, cur_y);
		}
		break;
	case indexed4:
		if (colors) {
			int bit = (3 - (idx & 3)) * 2;
			int ci  = (img->data[idx >> 2] >> bit) & 3;
			rgb = COLOR_TO_RGB888(colors[ci], cur_x, cur_y);
		}
		break;
	case indexed16:
		if (colors) {
			int bit = (1 - (idx & 1)) * 4;
			int ci  = (img->data[idx >> 1] >> bit) & 0xF;
			rgb = COLOR_TO_RGB888(colors[ci], cur_x, cur_y);
		}
		break;
	case rgb332: {
		uint8_t p = img->data[idx];
		rgb = ((uint32_t)((p >> 5) & 7) << (16+5)) | ((uint32_t)((p >> 2) & 7) << (8+5)) | ((uint32_t)(p & 3) << 6);
		break;
	}
	case rgb565: {
		uint16_t p = ((uint16_t)img->data[2*idx] << 8) | img->data[2*idx+1];
		rgb = ((uint32_t)(p >> 11) << (16+3)) | ((uint32_t)((p >> 5) & 0x3F) << (8+2)) | ((uint32_t)(p & 0x1F) << 3);
		break;
	}
	case rgb888:
		rgb = (uint32_t)img->data[3*idx] << 16 | (uint32_t)img->data[3*idx+1] << 8 | img->data[3*idx+2];
		break;
	default: break;
	}
	return rgb;
}

void disp_axs15231_command(uint8_t command, const uint8_t *args, int argn) {
	esp_lcd_panel_io_tx_param(m_io, command, args, argn);
}

bool disp_axs15231_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) {
	if (x + img->width > m_display_width || y + img->height > m_display_height) {
		return false;
	}

	uint16_t *buf = (uint16_t *)m_pix_buf;

    int px_min, py_min, target_w, target_h;

    if (m_rotation == 0) {
        px_min = x;
        py_min = y;
        target_w = img->width;
        target_h = img->height;
    } else if (m_rotation == 90) {
        px_min = (DISPLAY_WIDTH_PHYS - 1) - (y + img->height - 1);
        py_min = x;
        target_w = img->height;
        target_h = img->width;
    } else if (m_rotation == 180) {
        px_min = (DISPLAY_WIDTH_PHYS - 1) - (x + img->width - 1);
        py_min = (DISPLAY_HEIGHT_PHYS - 1) - (y + img->height - 1);
        target_w = img->width;
        target_h = img->height;
    } else {
        px_min = y;
        py_min = (DISPLAY_HEIGHT_PHYS - 1) - (x + img->width - 1);
        target_w = img->height;
        target_h = img->width;
    }

    uint32_t total_p = target_w * target_h;
    uint32_t pix_idx = 0;
    while (pix_idx < total_p) {
        uint32_t chunk_now = total_p - pix_idx;
        if (chunk_now > PIX_BUF_PIXELS) chunk_now = PIX_BUF_PIXELS;

        if (pix_idx + chunk_now < total_p) {
            chunk_now = (chunk_now / target_w) * target_w;
            if (chunk_now == 0) chunk_now = target_w;
        }

        for (uint32_t i = 0; i < chunk_now; i++) {
            uint32_t p_idx = pix_idx + i;
            int p_cur_x = p_idx % target_w;
            int p_cur_y = p_idx / target_w;
            
            int abs_px = px_min + p_cur_x;
            int abs_py = py_min + p_cur_y;
            int lx, ly;

            if (m_rotation == 0) {
                lx = abs_px;
                ly = abs_py;
            } else if (m_rotation == 90) {
                lx = abs_py;
                ly = (DISPLAY_WIDTH_PHYS - 1) - abs_px;
            } else if (m_rotation == 180) {
                lx = (DISPLAY_WIDTH_PHYS - 1) - abs_px;
                ly = (DISPLAY_HEIGHT_PHYS - 1) - abs_py;
            } else {
                lx = (DISPLAY_HEIGHT_PHYS - 1) - abs_py;
                ly = abs_px;
            }

            uint32_t src_idx = (ly - y) * img->width + (lx - x);
            buf[i] = to_disp_color(fetch_rgb888(img, src_idx, colors));
        }

        int p_start_y = py_min + (pix_idx / target_w);
        int p_end_y   = p_start_y + (chunk_now / target_w);
        if (p_end_y == p_start_y && chunk_now > 0) p_end_y++;
        
        esp_lcd_panel_draw_bitmap(m_panel, px_min, p_start_y, px_min + target_w, p_end_y, buf);
        pix_idx += chunk_now;
    }

	return true;
}

void disp_axs15231_clear(uint32_t color) {
	uint16_t c = to_disp_color(color);
	uint16_t *buf = (uint16_t *)m_pix_buf;

	for (int i = 0; i < PIX_BUF_PIXELS; i++) {
		buf[i] = c;
	}

	int total = DISPLAY_WIDTH_PHYS * DISPLAY_HEIGHT_PHYS;
	int sent = 0;
	while (sent < total) {
		int chunk = total - sent;
		if (chunk > PIX_BUF_PIXELS) chunk = PIX_BUF_PIXELS;

		int start_y = sent / DISPLAY_WIDTH_PHYS;
		int lines = chunk / DISPLAY_WIDTH_PHYS;
		if (lines == 0) lines = 1;

		esp_lcd_panel_draw_bitmap(m_panel, 0, start_y, DISPLAY_WIDTH_PHYS, start_y + lines, buf);
		sent += lines * DISPLAY_WIDTH_PHYS;
	}
}

static lbm_value ext_disp_cmd(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();
	if (argn > 1) {
		uint8_t cmd = (uint8_t)lbm_dec_as_u32(args[0]);
		uint8_t paras[32];
		int n = (int)argn - 1; if (n > 32) n = 32;
		for (int i = 0; i < n; i++) paras[i] = (uint8_t)lbm_dec_as_u32(args[i + 1]);
		disp_axs15231_command(cmd, paras, n);
		return ENC_SYM_TRUE;
	} else if (argn == 1) {
		disp_axs15231_command((uint8_t)lbm_dec_as_u32(args[0]), NULL, 0);
		return ENC_SYM_TRUE;
	}
	return ENC_SYM_TERROR;
}

static lbm_value ext_disp_orientation(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	uint32_t val = lbm_dec_as_u32(args[0]);
    
    uint8_t madctl = 0x08;
    disp_axs15231_command(0x36, &madctl, 1);

    if (val == 0) {
        m_rotation = 0;
        m_display_width = DISPLAY_WIDTH_PHYS;
        m_display_height = DISPLAY_HEIGHT_PHYS;
    } else if (val == 1) {
        m_rotation = 90;
        m_display_width = DISPLAY_HEIGHT_PHYS;
        m_display_height = DISPLAY_WIDTH_PHYS;
    } else if (val == 2) {
        m_rotation = 180;
        m_display_width = DISPLAY_WIDTH_PHYS;
        m_display_height = DISPLAY_HEIGHT_PHYS;
    } else if (val == 3) {
        m_rotation = 270;
        m_display_width = DISPLAY_HEIGHT_PHYS;
        m_display_height = DISPLAY_WIDTH_PHYS;
    }
	
	return ENC_SYM_TRUE;
}

void disp_axs15231_init(int pin_sd0, int pin_sd1, int pin_sd2, int pin_sd3,
		int pin_clk, int pin_cs, int pin_reset, int clock_mhz) {

	if (!m_pix_buf) {
		m_pix_buf = heap_caps_malloc(PIX_BUF_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
	}

	const spi_bus_config_t buscfg = AXS15231B_PANEL_BUS_QSPI_CONFIG(
			pin_clk, pin_sd0, pin_sd1, pin_sd2, pin_sd3,
			DISPLAY_WIDTH_PHYS * DISPLAY_HEIGHT_PHYS * 2);
	spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

	const esp_lcd_panel_io_spi_config_t io_cfg =
			AXS15231B_PANEL_IO_QSPI_CONFIG(pin_cs, NULL, NULL);
	esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &m_io);

	static axs15231b_vendor_config_t vendor_config = {
		.init_cmds      = lcd_init_cmds,
		.init_cmds_size = sizeof(lcd_init_cmds) / sizeof(axs15231b_lcd_init_cmd_t),
		.flags = {
			.use_qspi_interface = 1,
			.use_mipi_interface = 0,
		},
	};

	const esp_lcd_panel_dev_config_t panel_config = {
		.reset_gpio_num = pin_reset >= 0 ? (gpio_num_t)pin_reset : GPIO_NUM_NC,
		.rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_BGR,
		.bits_per_pixel = 16,
		.vendor_config  = &vendor_config,
	};

	esp_lcd_new_panel_axs15231b(m_io, &panel_config, &m_panel);
	esp_lcd_panel_reset(m_panel);
	esp_lcd_panel_init(m_panel);

	lbm_add_extension("ext-disp-cmd", ext_disp_cmd);
	lbm_add_extension("ext-disp-orientation", ext_disp_orientation);
}

void disp_axs15231_reset(void) {
	esp_lcd_panel_reset(m_panel);
	esp_lcd_panel_init(m_panel);
	disp_axs15231_clear(0);
}