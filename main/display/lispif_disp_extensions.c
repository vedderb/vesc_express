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


#include "lispif_disp_extensions.h"
#include "lispif.h"
#include "lbm_utils.h"
#include "lbm_custom_type.h"
#include "commands.h"

#include "display/disp_sh8501b.h"
#include "display/disp_ili9341.h"
#include "display/disp_ssd1306.h"
#include "display/tjpgd.h"

#include <math.h>

static const uint8_t cos_tab_256[] = {
		255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251,
		250, 250, 249, 248, 246, 245, 244, 243, 241, 240, 238, 237, 235, 234,
		232, 230, 228, 226, 224, 222, 220, 218, 215, 213, 211, 208, 206, 203,
		201, 198, 196, 193, 190, 188, 185, 182, 179, 176, 173, 170, 167, 165,
		162, 158, 155, 152, 149, 146, 143, 140, 137, 134, 131, 127, 124, 121,
		118, 115, 112, 109, 106, 103, 100, 97, 93, 90, 88, 85, 82, 79, 76, 73,
		70, 67, 65, 62, 59, 57, 54, 52, 49, 47, 44, 42, 40, 37, 35, 33, 31, 29,
		27, 25, 23, 21, 20, 18, 17, 15, 14, 12, 11, 10, 9, 7, 6, 5, 5, 4, 3, 2,
		2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 9, 10,
		11, 12, 14, 15, 17, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35, 37, 40, 42,
		44, 47, 49, 52, 54, 57, 59, 62, 65, 67, 70, 73, 76, 79, 82, 85, 88, 90,
		93, 97, 100, 103, 106, 109, 112, 115, 118, 121, 124, 128, 131, 134, 137,
		140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173, 176, 179,
		182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215,
		218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241,
		243, 244, 245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254,
		254, 255, 255, 255
};

uint32_t lispif_disp_rgb888_from_color(color_t color, int x, int y) {
	switch (color.type) {
	case COLOR_REGULAR:
		return color.color1;

	case COLOR_GRADIENT_X:
	case COLOR_GRADIENT_Y: {
		uint32_t res;
		uint32_t r1 = color.color1 >> 16;
		uint32_t g1 = (color.color1) >> 8 & 0xFF;
		uint32_t b1 = color.color1 & 0xff;

		uint32_t r2 = color.color2 >> 16;
		uint32_t g2 = (color.color2) >> 8 & 0xFF;
		uint32_t b2 = color.color2 & 0xff;

		int pos = color.type == COLOR_GRADIENT_X ? x : y;
		int tab_pos = ((pos * 256) / color.param1 + color.param2) % 256;
		int tab_val = cos_tab_256[tab_pos];

		uint32_t r = (r1 * tab_val + r2 * (255 - tab_val)) / 255;
		uint32_t g = (g1 * tab_val + g2 * (255 - tab_val)) / 255;
		uint32_t b = (b1 * tab_val + b2 * (255 - tab_val)) / 255;

		res = r << 16 | g << 8 | b;
		return res;
	}

	default:
		return 0;
	}
}

static const char *image_buffer_desc = "Image-Buffer";
static const char *color_desc = "Color";

static lbm_uint symbol_indexed2 = 0;
static lbm_uint symbol_indexed4 = 0;
static lbm_uint symbol_rgb332 = 0;
static lbm_uint symbol_rgb565 = 0;
static lbm_uint symbol_rgb888 = 0;

static lbm_uint symbol_thickness = 0;
static lbm_uint symbol_filled = 0;
static lbm_uint symbol_rounded = 0;
static lbm_uint symbol_dotted = 0;
static lbm_uint symbol_scale = 0;
static lbm_uint symbol_rotate = 0;

static lbm_uint symbol_regular = 0;
static lbm_uint symbol_gradient_x = 0;
static lbm_uint symbol_gradient_y = 0;
static lbm_uint symbol_gradient_x_pre = 0;
static lbm_uint symbol_gradient_y_pre = 0;

static color_format_t sym_to_color_format(lbm_value v) {
	lbm_uint s = lbm_dec_sym(v);
	if (s == symbol_indexed2) return indexed2;
	if (s == symbol_indexed4) return indexed4;
	if (s == symbol_rgb332) return rgb332;
	if (s == symbol_rgb565) return rgb565;
	if (s == symbol_rgb888) return rgb888;
	return format_not_supported;

}

static bool image_buffer_destructor(lbm_uint value) {
	image_buffer_t *img = (image_buffer_t*)value;
	lbm_free((void*)img->data);
	lbm_free((void*)img);
	return true;
}

static uint32_t image_dims_to_size_bytes(color_format_t fmt, uint16_t width, uint16_t height) {
	uint32_t num_pix = (uint32_t)width * (uint32_t)height;
	switch(fmt) {
	case indexed2:
		if (num_pix % 8 != 0) return (num_pix / 8) + 1;
		else return (num_pix / 8);
		break;
	case indexed4:
		if (num_pix % 4 != 0) return (num_pix / 4) + 1;
		else return (num_pix / 4);
		break;
	case rgb332:
		return num_pix;
		break;
	case rgb565:
		return num_pix * 2;
		break;
	case rgb888:
		return num_pix * 3;
	default:
		return 0;
	}

}

static lbm_value image_buffer_lift(uint8_t *buf, uint8_t buf_offset, color_format_t fmt, uint16_t width, uint16_t height) {
	image_buffer_t *img = lbm_malloc(sizeof(image_buffer_t));
	if (!img) {
		return ENC_SYM_MERROR;
	}

	lbm_value res;
	if (!lbm_custom_type_create((lbm_uint) img, image_buffer_destructor,
			image_buffer_desc, &res)) {
		lbm_free(img);
		return ENC_SYM_MERROR;
	}

	img->data_offset = buf_offset;
	img->data = buf;
	img->fmt = fmt;
	img->width = width;
	img->height = height;

	return res;
}

static bool color_destructor(lbm_uint value) {
	color_t *color = (color_t*)value;
	if (color->precalc) {
		lbm_free((void*)color->precalc);
	}
	lbm_free((void*)color);
	return true;
}

static lbm_value color_allocate(COLOR_TYPE type, uint32_t color1, uint32_t color2, int param1, int param2) {
	color_t *color = lbm_malloc(sizeof(color_t));
	if (!color) {
		return ENC_SYM_MERROR;
	}

	uint32_t *pre = 0;
	if (type == COLOR_PRE_X || type == COLOR_PRE_Y) {
		pre = lbm_malloc(COLOR_PRECALC_LEN * sizeof(uint32_t));
		if (!pre) {
			lbm_free(color);
			return ENC_SYM_MERROR;
		}
	}

	lbm_value res;
	if (!lbm_custom_type_create((lbm_uint)color,
			color_destructor, color_desc, &res)) {
		lbm_free(color);
		if (pre) {
			lbm_free(pre);
		}
		return ENC_SYM_MERROR;
	}

	color->type = type;
	color->color1 = color1;
	color->color2 = color2;
	color->param1 = param1;
	color->param2 = param2;
	color->precalc = pre;

	if (pre) {
		COLOR_TYPE type_old = color->type;
		if (type == COLOR_PRE_X) {
			color->type = COLOR_GRADIENT_X;
		} else if (type == COLOR_PRE_Y) {
			color->type = COLOR_GRADIENT_Y;
		}

		for (int i = 0;i < COLOR_PRECALC_LEN;i++) {
			pre[i] = lispif_disp_rgb888_from_color(*color, i, i);
		}

		color->type = type_old;
	}

	return res;
}

static lbm_value image_buffer_allocate(color_format_t fmt, uint16_t width, uint16_t height) {
	uint32_t size_bytes = image_dims_to_size_bytes(fmt, width, height);

	uint8_t *buf = lbm_malloc(size_bytes);
	if (!buf) {
		return ENC_SYM_MERROR;
	}
	memset(buf, 0, size_bytes);
	lbm_value res = image_buffer_lift(buf, 0, fmt, width, height);
	if (lbm_is_symbol(res)) { /* something is wrong, free */
		lbm_free(buf);
	}
	return res;
}

// Exported interface

bool lispif_disp_is_image_buffer(lbm_value v) {
	return ((lbm_uint)lbm_get_custom_descriptor(v) == (lbm_uint)image_buffer_desc);
}

bool lispif_disp_is_color(lbm_value v) {
	return ((lbm_uint)lbm_get_custom_descriptor(v) == (lbm_uint)color_desc);
}

// Register symbols

static bool register_symbols(void) {
	bool res = true;
	res = res && lbm_add_symbol_const("indexed2", &symbol_indexed2);
	res = res && lbm_add_symbol_const("indexed4", &symbol_indexed4);
	res = res && lbm_add_symbol_const("rgb332", &symbol_rgb332);
	res = res && lbm_add_symbol_const("rgb565", &symbol_rgb565);
	res = res && lbm_add_symbol_const("rgb888", &symbol_rgb888);

	res = res && lbm_add_symbol_const("thickness", &symbol_thickness);
	res = res && lbm_add_symbol_const("filled", &symbol_filled);
	res = res && lbm_add_symbol_const("rounded", &symbol_rounded);
	res = res && lbm_add_symbol_const("dotted", &symbol_dotted);
	res = res && lbm_add_symbol_const("scale", &symbol_scale);
	res = res && lbm_add_symbol_const("rotate", &symbol_rotate);

	res = res && lbm_add_symbol_const("regular", &symbol_regular);
	res = res && lbm_add_symbol_const("gradient_x", &symbol_gradient_x);
	res = res && lbm_add_symbol_const("gradient_y", &symbol_gradient_y);
	res = res && lbm_add_symbol_const("gradient_x_pre", &symbol_gradient_x_pre);
	res = res && lbm_add_symbol_const("gradient_y_pre", &symbol_gradient_y_pre);
	return res;
}

// Internal functions

static inline void norm_angle(float *angle) {
	while (*angle < -M_PI) { *angle += 2.0 * M_PI; }
	while (*angle >=  M_PI) { *angle -= 2.0 * M_PI; }
}

static uint8_t rgb888to332(uint32_t rgb) {
	uint8_t r = (uint8_t)(rgb >> (16 + 5));
	uint8_t g = (uint8_t)(rgb >> (8 + 5));
	uint8_t b = (uint8_t)(rgb >> 6);
	r <<= 5;
	g = (g & 0x7) << 2;  ;
	b = (b & 0x3);
	uint8_t rgb332 = r | g | b;
	return rgb332;
}

static uint16_t rgb888to565(uint32_t rgb) {
	uint16_t r = (uint16_t)(rgb >> (16 + 3));
	uint16_t g = (uint16_t)(rgb >> (8 + 2));
	uint16_t b = (uint16_t)(rgb >> 3);
	r <<= 11;
	g = (g & 0x3F) << 5;
	b = (b & 0x1F);
	uint16_t rgb565 = r | g | b;
	return rgb565;
}

static uint32_t rgb332to888(uint8_t rgb) {
	uint32_t r = (uint32_t)((rgb>>5) & 0x7);
	uint32_t g = (uint32_t)((rgb>>2) & 0x7);
	uint32_t b = (uint32_t)(rgb & 0x3);
	uint32_t rgb888 = r << (16 + 5) | g << (8 + 5) | b << 6;
	return rgb888;
}

static uint32_t  rgb565to888(uint16_t rgb) {
	uint32_t r = (uint32_t)(rgb >> 11);
	uint32_t g = (uint32_t)((rgb >> 5) & 0x3F);
	uint32_t b = (uint32_t)(rgb & 0x1F);
	uint32_t rgb888 = r << (16 + 3) | g << (8 + 2) | b << 3;
	return rgb888;
}

static void image_buffer_clear(image_buffer_t *img, uint32_t cc) {
	uint32_t img_size = (uint32_t)img->width * img->height;
	switch (img->fmt) {
	case indexed2: {
		int extra = img_size & 0x7;
		int bytes = (img_size >> 3) + (extra ? 1 : 0);
		uint8_t c8 = (cc & 1) ? 0xFFFF : 0x0;
		memset(img->data+img->data_offset, c8, bytes);
	}
	break;
	case indexed4: {
		static const uint8_t index4_table[4] = {0x00, 0x55, 0xAA, 0xFF};
		int extra = img_size & 0x3;
		int bytes = (img_size >> 2) + (extra ? 1 : 0);
		uint8_t ix = (cc & 0x3);
		memset(img->data+img->data_offset, index4_table[ix], bytes);
	}
	break;
	case rgb332: {
		memset(img->data+img->data_offset, rgb888to332(cc), img_size);
	}
	break;
	case rgb565: {
		uint16_t c = rgb888to565(cc);
		uint8_t *dp = (uint8_t*)img->data+img->data_offset;
		for (int i = 0; i < img_size/2; i +=2) {
			dp[i] = (uint8_t)c >> 8;
			dp[i+1] = (uint8_t)c;
		}
	}
	break;
	case rgb888: {
		uint8_t *dp = (uint8_t*)img->data+img->data_offset;
		for (int i = 0; i < img_size * 3; i+= 3) {
			dp[i]   = (uint8_t)cc >> 16;
			dp[i+1] = (uint8_t)cc >> 8;
			dp[i+2] = (uint8_t)cc;
		}
	}
	break;
	default:
		break;
	}
}

static uint8_t indexed4_mask[4] = {0x03, 0x0C, 0x30, 0xC0};
static uint8_t indexed4_shift[4] = {0, 2, 4, 6};

static void putpixel(image_buffer_t* img, uint16_t x, uint16_t y, uint32_t c) {
	uint16_t w = img->width;
	uint16_t h = img->height;
	if (x < w && y < h) {
		uint8_t *data = img->data+img->data_offset;
		switch(img->fmt) {
		case indexed2: {
			uint32_t pos = y * w + x;
			uint32_t byte = pos >> 3;
			uint32_t bit  = 7 - (pos & 0x7);
			if (c) {
				data[byte] |= (1 << bit);
			} else {
				data[byte] &= ~(1 << bit);
			}
			break;
		}
		case indexed4: {
			int pos = y*w + x;
			uint32_t byte = pos >> 2;
			uint32_t ix  = 3 - (pos & 0x3);
			data[byte] =  (data[byte] & ~indexed4_mask[ix]) | c << indexed4_shift[ix];
			break;
		}
		case rgb332: {
			int pos = y*w + x;
			data[pos] = rgb888to332(c);
			break;
		}
		case rgb565: {
			int pos = y*(w<<1) + (x<<1) ;
			uint16_t color = rgb888to565(c);
			data[pos] = (uint8_t)color >> 8;
			data[pos+1] = (uint8_t)color;
			break;
		}
		case rgb888: {
			int pos = y*(w*3) + (x*3);
			data[pos] = (uint8_t)c>>16;
			data[pos+1] = (uint8_t)c>>8;
			data[pos+2] = (uint8_t)c;
			break;
		}
		default:
			break;
		}
	}
}

static uint32_t getpixel(image_buffer_t* img, uint16_t x, uint16_t y) {
	uint16_t w = img->width;
	uint16_t h = img->height;
	if (x < w && y < h) {
		uint8_t *data = img->data+img->data_offset;
		switch(img->fmt) {
		case indexed2: {
			uint32_t pos = y * w + x;
			uint32_t byte = pos >> 3;
			uint32_t bit  = 7 - (pos & 0x7);
			return (uint32_t)(data[byte] >> bit) & 0x1;
		}
		case indexed4: {
			int pos = y*w + x;
			uint32_t byte = pos >> 2;
			uint32_t ix  = 3 - (pos & 0x3);
			return (uint32_t)((data[byte] & indexed4_mask[ix]) >> indexed4_shift[ix]);
		}
		case rgb332: {
			int pos = y*w + x;
			return rgb332to888(data[pos]);
		}
		case rgb565: {
			int pos = y*(w<<1) + (x<<1);
			uint16_t c = ((uint16_t)data[pos] << 8) | (uint16_t)data[pos+1];
			return rgb565to888(c);
		}
		case rgb888: {
			int pos = y*(w*3) + (x*3);
			uint32_t r = data[pos];
			uint32_t g = data[pos+1];
			uint32_t b = data[pos+2];
			return (r << 16 | g << 8 | b);
		}
		default:
			break;
		}
	}
	return 0;
}

static void h_line(image_buffer_t* img, int16_t x, int16_t y, uint16_t len, uint32_t c) {
	for (int i = 0; i < len; i ++) {
		putpixel(img, x+i, y, c);
	}
}

static void v_line(image_buffer_t* img, int16_t x, int16_t y, uint16_t len, uint32_t c) {
	for (int i = 0; i < len; i ++) {
		putpixel(img, x, y+i, c);
	}
}

static void fill_circle(image_buffer_t *img, int x, int y, int radius, uint32_t color) {
	switch (radius) {
	case 0:
		putpixel(img, x, y, color);
		break;

	case 1:
		putpixel(img, x - 1, y, color);
		putpixel(img, x, y, color);
		break;

	case 2:
		putpixel(img, x - 1, y - 1, color);
		putpixel(img, x, y - 1, color);
		putpixel(img, x - 2, y, color);
		putpixel(img, x - 1, y, color);
		putpixel(img, x, y, color);
		putpixel(img, x + 1, y, color);
		putpixel(img, x - 1, y + 1, color);
		putpixel(img, x, y + 1, color);
		break;

	case 3:
		h_line(img, x - 2, y - 2, 4, color);
		h_line(img, x - 2, y - 1, 4, color);
		h_line(img, x - 3, y, 6, color);
		h_line(img, x - 2, y + 1, 4, color);
		h_line(img, x - 2, y + 2, 4, color);
		break;

	case 4:
		h_line(img, x - 2, y - 3, 4, color);
		h_line(img, x - 3, y - 2, 6, color);
		h_line(img, x - 3, y - 1, 6, color);
		h_line(img, x - 4, y, 8, color);
		h_line(img, x - 3, y + 1, 6, color);
		h_line(img, x - 3, y + 2, 6, color);
		h_line(img, x - 2, y + 3, 4, color);
		break;

	default: {
		int r2 = radius * radius;
		for(int y1 = -radius;y1 <= 0;y1++) {
			for(int x1 =- radius;x1 <= 0;x1++) {
				if(x1 * x1 + y1 * y1 <= r2) {
					h_line(img, x + x1, y + y1, 2 * (-x1), color);
					h_line(img, x + x1, y - y1, 2 * (-x1), color);
					break;
				}
			}
		}
	} break;
	}
}

static void circle(image_buffer_t *img, int x, int y, int radius, int thickness, uint32_t color) {
	int x0 = 0;
	int y0 = radius;
	int d = 5 - 4*radius;
	int da = 12;
	int db = 20 - 8*radius;

	if (thickness < 2) {
		while (x0 < y0) {
			putpixel(img, x + x0, y + y0, color);
			putpixel(img, x + x0, y - y0, color);
			putpixel(img, x - x0, y + y0, color);
			putpixel(img, x - x0, y - y0, color);
			putpixel(img, x + y0, y + x0, color);
			putpixel(img, x + y0, y - x0, color);
			putpixel(img, x - y0, y + x0, color);
			putpixel(img, x - y0, y - x0, color);
			if (d < 0) { d = d + da; db = db+8; }
			else  { y0 = y0 - 1; d = d+db; db = db + 16; }
			x0 = x0+1;
			da = da + 8;
		}
	} else {
		while (x0 < y0) {
			fill_circle(img, x + x0, y + y0, thickness, color);
			fill_circle(img, x + x0, y - y0, thickness, color);
			fill_circle(img, x - x0, y + y0, thickness, color);
			fill_circle(img, x - x0, y - y0, thickness, color);
			fill_circle(img, x + y0, y + x0, thickness, color);
			fill_circle(img, x + y0, y - x0, thickness, color);
			fill_circle(img, x - y0, y + x0, thickness, color);
			fill_circle(img, x - y0, y - x0, thickness, color);
			if (d < 0) { d = d + da; db = db+8; }
			else  { y0 = y0 - 1; d = d+db; db = db + 16; }
			x0 = x0+1;
			da = da + 8;
		}
	}
}

// TODO: This should be more efficient
// http://homepages.enterprise.net/murphy/thickline/index.html
// https://github.com/ArminJo/STMF3-Discovery-Demos/blob/master/lib/BlueDisplay/LocalGUI/ThickLine.hpp
static void line(image_buffer_t *img, int x0, int y0, int x1, int y1, int thickness, int dot1, int dot2, uint32_t c) {
	int dx = abs(x1 - x0);
	int sx = x0 < x1 ? 1 : -1;
	int dy = -abs(y1 - y0);
	int sy = y0 < y1 ? 1 : -1;
	int error = dx + dy;

	if (dot1 > 0) {
		// These are used to deal with consecutive calls with
		// possibly overlapping pixels.
		static int dotcnt = 0;
		static int x_last = 0;
		static int y_last = 0;

		while (true) {
			if (dotcnt <= dot1) {
				if (thickness > 1) {
					fill_circle(img, x0, y0, thickness, c);
				} else {
					putpixel(img, x0, y0, c);
				}
			}

			if (x0 != x_last || y0 != y_last) {
				dotcnt++;
			}

			x_last = x0;
			y_last = y0;

			if (dotcnt >= (dot1 + dot2)) {
				dotcnt = 0;
			}

			if (x0 == x1 && y0 == y1) {
				break;
			}
			if ((error * 2) >= dy) {
				if (x0 == x1) {
					break;
				}
				error += dy;
				x0 += sx;
			}
			if ((error * 2) <= dx) {
				if (y0 == y1) {
					break;
				}
				error += dx;
				y0 += sy;
			}
		}
	} else {
		while (true) {
			if (thickness > 1) {
				fill_circle(img, x0, y0, thickness, c);
			} else {
				putpixel(img, x0, y0, c);
			}

			if (x0 == x1 && y0 == y1) {
				break;
			}
			if ((error * 2) >= dy) {
				if (x0 == x1) {
					break;
				}
				error += dy;
				x0 += sx;
			}
			if ((error * 2) <= dx) {
				if (y0 == y1) {
					break;
				}
				error += dx;
				y0 += sy;
			}
		}
	}
}

static void rectangle(image_buffer_t *img, int x, int y, int width, int height,
		bool fill, int thickness, int dot1, int dot2, uint32_t color) {
	if (fill) {
		for (int i = y; i < (y + height);i++) {
			h_line(img, x, i, width, color);
		}
	} else {
		if (thickness <= 1 && dot1 == 0) {
			h_line(img, x, y, width, color);
			h_line(img, x, y + height, width, color);
			v_line(img, x, y, height, color);
			v_line(img, x + width, y, height, color);
		} else {
			line(img, x, y, x + width, y, thickness, dot1, dot2, color);
			line(img, x, y + height, x + width, y + height, thickness, dot1, dot2, color);
			line(img, x, y, x, y + height, thickness, dot1, dot2, color);
			line(img, x + width, y, x + width, y + height, thickness, dot1, dot2, color);
		}
	}
}

#define NMIN(a, b) ((a) < (b) ? (a) : (b))
#define NMAX(a, b) ((a) > (b) ? (a) : (b))

static bool triangle_edge(int ax, int ay, int bx, int by, int cx, int cy) {
	return ((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) >= 0;
}

static void fill_triangle(image_buffer_t *img, int x0, int y0,
		int x1, int y1, int x2, int y2, uint32_t color) {
	int x_min = NMIN(x0, NMIN(x1, x2));
	int x_max = NMAX(x0, NMAX(x1, x2));
	int y_min = NMIN(y0, NMIN(y1, y2));
	int y_max = NMAX(y0, NMAX(y1, y2));

	for (int y = y_min;y <= y_max;y++) {
		for (int x = x_min;x <= x_max;x++) {
			bool w0 = triangle_edge(x1, y1, x2, y2, x, y);
			bool w1 = triangle_edge(x2, y2, x0, y0, x, y);
			bool w2 = triangle_edge(x0, y0, x1, y1, x, y);

			if (w0 && w1 && w2) {
				putpixel(img, x, y, color);
			}
		}
	}
}

static void arc(image_buffer_t *img, int x, int y, int rad, float ang_start, float ang_end,
		int thickness, bool filled, int dot1, int dot2, bool sector, bool segment, uint32_t color) {
	ang_start *= M_PI / 180.0;
	ang_end *= M_PI / 180.0;

	norm_angle(&ang_start);
	norm_angle(&ang_end);

	float ang_range = ang_end - ang_start;

	if (ang_range < 0.0) {
		ang_range += 2.0 * M_PI;
	}

	float steps = 40.0 * ang_range * (0.5 / M_PI);

	float ang_step = ang_range / steps;
	float sa = sinf(ang_step);
	float ca = cosf(ang_step);

	float px_start = cosf(ang_start) * (float)rad;
	float py_start = sinf(ang_start) * (float)rad;

	float px = px_start;
	float py = py_start;

	for (int i = 0;i < steps;i++) {
		float px_before = px;
		float py_before = py;

		px = px * ca - py * sa;
		py = py * ca + px_before * sa;

		if (filled) {
			if (sector) {
				fill_triangle(img,
						x + px_before, y + py_before,
						x + px, y + py,
						x, y,
						color);
			} else {
				fill_triangle(img,
						x + px_before, y + py_before,
						x + px, y + py,
						x + px_start, y + py_start,
						color);
			}
		} else {
			line(img, x + px_before, y + py_before,
					x + px, y + py, thickness, dot1, dot2, color);
		}
	}

	if (!filled && sector) {
		line(img, x + px, y + py,
				x, y,
				thickness, dot1, dot2, color);
		line(img, x, y,
				x + px_start, y + py_start,
				thickness, dot1, dot2, color);
	}

	if (!filled && segment) {
		line(img, x + px, y + py,
				x + px_start, y + py_start,
				thickness, dot1, dot2, color);
	}
}

static void img_putc(image_buffer_t *img, int x, int y, int fg, int bg, uint8_t *font_data, uint8_t ch) {
	uint8_t w = font_data[0];
	uint8_t h = font_data[1];
	uint8_t char_num = font_data[2];
//	uint8_t bits_per_pixel = font_data[3];

	if (char_num == 10) {
		ch -= '0';
	} else {
		ch -= ' ';
	}

	if (ch >= char_num) {
		return;
	}

	for (int i = 0; i < w; i ++) {
		for (int j = 0; j < h; j ++) {
			int f_ind = (j * w + i);
			int f_pos = 4 + ch * (w * h) / 8 + (f_ind / 8);
			int bit_pos = f_ind % 8;
			int bit = font_data[f_pos] & (1 << bit_pos);
			if (bit || bg >= 0) {
				putpixel(img, x+i, y+j, bit ? fg : bg);
			}
		}
	}
}

static void blit_rot_scale(
		image_buffer_t *img_dest,
		image_buffer_t *img_src,
		int x, int y, // Where on display
		float xr, float yr, // Pixel to rotate around
		float rot, // Rotation angle in degrees
		float scale, // Scale factor
		int32_t transparent_color) {

	int src_w = img_src->width;
	int src_h = img_src->height;
	int des_w = img_dest->width;
	int des_h = img_dest->height;

	int des_x_start = 0;
	int des_y_start = 0;
	int des_x_end = (des_x_start + des_w);
	int des_y_end = (des_y_start + des_h);

	if (des_x_start < 0) des_x_start = 0;
	if (des_x_end > des_w) des_x_end = des_w;
	if (des_y_start < 0) des_y_start = 0;
	if (des_y_end > des_h) des_y_end = des_h;

	if (rot == 0.0 && scale == 1.0) {
		if (x > 0) des_x_start += x;
		if (y > 0) des_y_start += y;
		if ((des_x_end - x) > src_w) des_x_end = src_w + x;
		if ((des_y_end - y) > src_h) des_y_end = src_h + y;

		for (int j = des_y_start; j < des_y_end; j++) {
			for (int i = des_x_start; i < des_x_end; i++) {
				int px = i - x;
				int py = j - y;

				if (px >= 0 && px < src_w && py >= 0 && py < src_h) {
					uint32_t p = getpixel(img_src, px, py);

					if (p != (uint32_t) transparent_color) {
						putpixel(img_dest, i, j, p);
					}
				}
			}
		}
	} else if (rot == 0.0) {
		xr *= scale;
		yr *= scale;

		const int fp_scale = 1000;

		int xr_i = xr;
		int yr_i = yr;
		int scale_i = scale * (float) fp_scale;

		for (int j = des_y_start; j < des_y_end; j++) {
			for (int i = des_x_start; i < des_x_end; i++) {
				int px = (i - x - xr_i) * fp_scale;
				int py = (j - y - yr_i) * fp_scale;

				px += xr_i * fp_scale;
				py += yr_i * fp_scale;

				px /= scale_i;
				py /= scale_i;

				if (px >= 0 && px < src_w && py >= 0 && py < src_h) {
					uint32_t p = getpixel(img_src, px, py);

					if (p != (uint32_t) transparent_color) {
						putpixel(img_dest, i, j, p);
					}
				}
			}
		}
	} else {
		float sr = sinf(-rot * M_PI / 180.0f);
		float cr = cosf(-rot * M_PI / 180.0f);

		xr *= scale;
		yr *= scale;

		const int fp_scale = 1000;

		int sr_i = sr * fp_scale;
		int cr_i = cr * fp_scale;
		int xr_i = xr;
		int yr_i = yr;
		int scale_i = scale * (float) fp_scale;

		for (int j = des_y_start; j < des_y_end; j++) {
			for (int i = des_x_start; i < des_x_end; i++) {
				int px = (i - x - xr_i) * cr_i + (j - y - yr_i) * sr_i;
				int py = -(i - x - xr_i) * sr_i + (j - y - yr_i) * cr_i;

				px += xr_i * fp_scale;
				py += yr_i * fp_scale;

				px /= scale_i;
				py /= scale_i;

				if (px >= 0 && px < src_w && py >= 0 && py < src_h) {
					uint32_t p = getpixel(img_src, px, py);

					if (p != (uint32_t) transparent_color) {
						putpixel(img_dest, i, j, p);
					}
				}
			}
		}
	}
}

// Extensions

#define ATTR_MAX_ARGS	3
#define ARG_MAX_NUM		8

typedef struct {
	bool is_valid;
	uint16_t arg_num;
	lbm_value args[ATTR_MAX_ARGS];
} attr_t;

typedef struct {
	bool is_valid;
	image_buffer_t *img;
	lbm_value args[ARG_MAX_NUM];
	attr_t attr_thickness;
	attr_t attr_filled;
	attr_t attr_rounded;
	attr_t attr_dotted;
	attr_t attr_scale;
	attr_t attr_rotate;
} img_args_t;

static img_args_t decode_args(lbm_value *args, lbm_uint argn, int num_expected) {
	img_args_t res;
	memset(&res, 0, sizeof(res));

	if (!lispif_disp_is_image_buffer(args[0])) {
		return res;
	}

	res.img = (image_buffer_t*)lbm_get_custom_value(args[0]);

	int num_dec = 0;
	for (int i = 1;i < argn;i++) {
		if (!lbm_is_number(args[i]) && !lbm_is_cons(args[i])) {
			return res;
		}

		if (lbm_is_number(args[i])) {
			res.args[num_dec] = args[i];
			num_dec++;

			if (num_dec > ARG_MAX_NUM) {
				return res;
			}
		} else {
			lbm_value curr = args[i];
			int attr_ind = 0;
			attr_t *attr_now = 0;
			while (lbm_is_cons(curr)) {
				lbm_value  arg = lbm_car(curr);

				if (attr_ind == 0) {
					if (!lbm_is_symbol(arg)) {
						return res;
					}

					if (lbm_dec_sym(arg) == symbol_thickness) {
						attr_now = &res.attr_thickness;
						attr_now->arg_num = 1;
					} else if (lbm_dec_sym(arg) == symbol_filled) {
						attr_now = &res.attr_filled;
						attr_now->arg_num = 0;
					} else if (lbm_dec_sym(arg) == symbol_rounded) {
						attr_now = &res.attr_rounded;
						attr_now->arg_num = 1;
					} else if (lbm_dec_sym(arg) == symbol_dotted) {
						attr_now = &res.attr_dotted;
						attr_now->arg_num = 2;
					} else if (lbm_dec_sym(arg) == symbol_scale) {
						attr_now = &res.attr_scale;
						attr_now->arg_num = 1;
					} else if (lbm_dec_sym(arg) == symbol_rotate) {
						attr_now = &res.attr_rotate;
						attr_now->arg_num = 3;
					} else {
						return res;
					}
				} else {
					if (!lbm_is_number(arg)) {
						return res;
					}

					attr_now->args[attr_ind - 1] = arg;
				}

				attr_ind++;
				if (attr_ind > (ATTR_MAX_ARGS + 1)) {
					return res;
				}

				curr = lbm_cdr(curr);
			}

			if ((attr_ind - 1) == attr_now->arg_num) {
				attr_now->is_valid = true;
			} else {
				return res;
			}
		}
	}

	if (num_dec != num_expected) {
		return res;
	}

	res.is_valid = true;
	return res;
}

static lbm_value ext_image_dims(lbm_value *args, lbm_uint argn) {
	img_args_t arg_dec = decode_args(args, argn, 0);

	if (!arg_dec.is_valid) {
		return ENC_SYM_TERROR;
	}

	lbm_value dims = lbm_heap_allocate_list(2);
	if (lbm_is_symbol(dims)) {
		return dims;
	}
	lbm_value curr = dims;
	lbm_set_car(curr, lbm_enc_i(arg_dec.img->width));
	curr = lbm_cdr(curr);
	lbm_set_car(curr, lbm_enc_i(arg_dec.img->height));
	return dims;
}

static lbm_value ext_image_buffer(lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_TERROR;

	if (argn == 3 &&
		lbm_is_symbol(args[0]) &&
		lbm_is_number(args[1]) &&
		lbm_is_number(args[2])) {

		color_format_t fmt = sym_to_color_format(args[0]);
		if (fmt != format_not_supported) {
			res = image_buffer_allocate(fmt, lbm_dec_as_u32(args[1]), lbm_dec_as_u32(args[2]));
		}
	}
	return res;
}

static lbm_value ext_image_buffer_from_bin(lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_TERROR;

	if (argn == 1 &&
		lbm_is_array(args[0])) {

		lbm_value arr = args[0];
		//color_format_t fmt = sym_to_color_format(args[1]);
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(arr);
		uint8_t *data = (uint8_t*)array->data;
		uint16_t w = ((uint16_t)data[0]) << 8 | ((uint16_t)data[1]);
		uint16_t h = ((uint16_t)data[2]) << 8 | ((uint16_t)data[3]);
		uint8_t bits = data[4];

		color_format_t fmt;
		switch(bits) {
		case 1: fmt = indexed2; break;
		case 2: fmt = indexed4; break;
		case 8: fmt = rgb332; break;
		case 16: fmt = rgb565; break;
		case 24: fmt = rgb888; break;
		default: fmt = format_not_supported; break; // return ENC_SYM_TERROR;
		}

		res = image_buffer_lift((uint8_t*)array->data, 5, fmt,  w, h);
		if (!lbm_is_symbol(res)) {  // Take ownership of array
			lbm_set_car(arr,ENC_SYM_NIL);
			lbm_set_cdr(arr,ENC_SYM_NIL);
			lbm_set_ptr_type(arr, LBM_TYPE_CONS);
		}
	}
	return res;
}

static lbm_value ext_color(lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_TERROR;

	if (argn >= 2 && argn <= 5 &&
			lbm_is_symbol(args[0]) &&
			lbm_is_number(args[1])) {

		uint32_t color1 = lbm_dec_as_i32(args[1]);

		uint32_t color2 = 0;
		if (argn >= 3) {
			if (lbm_is_number(args[2])) {
				color2 = lbm_dec_as_i32(args[2]);
			} else {
				return ENC_SYM_TERROR;
			}
		}

		uint32_t param1 = 0;
		if (argn >= 4) {
			if (lbm_is_number(args[3])) {
				param1 = lbm_dec_as_i32(args[3]);
			} else {
				return ENC_SYM_TERROR;
			}
		}

		uint32_t param2 = 0;
		if (argn >= 5) {
			if (lbm_is_number(args[4])) {
				param2 = lbm_dec_as_i32(args[4]);
			} else {
				return ENC_SYM_TERROR;
			}
		}

		COLOR_TYPE t;
		if (lbm_dec_sym(args[0]) == symbol_regular) {
			t = COLOR_REGULAR;
		} else if (lbm_dec_sym(args[0]) == symbol_gradient_x) {
			t = COLOR_GRADIENT_X;
		} else if (lbm_dec_sym(args[0]) == symbol_gradient_y) {
			t = COLOR_GRADIENT_Y;
		} else if (lbm_dec_sym(args[0]) == symbol_gradient_x_pre) {
			t = COLOR_PRE_X;
		} else if (lbm_dec_sym(args[0]) == symbol_gradient_y_pre) {
			t = COLOR_PRE_Y;
		} else {
			return ENC_SYM_TERROR;
		}

		res = color_allocate(t, color1, color2, param1, param2);
	}

	return res;
}

static lbm_value ext_color_setpre(lbm_value *args, lbm_uint argn) {
	if (argn != 3 || !lispif_disp_is_color(args[0]) ||
			!lbm_is_number(args[1]) || !lbm_is_number(args[2])) {
		return ENC_SYM_TERROR;
	}

	color_t *color = (color_t*)lbm_get_custom_value(args[0]);

	uint32_t pos = lbm_dec_as_u32(args[1]);
	int new_color = lbm_dec_as_i32(args[2]);

	if (color->precalc == 0 || pos >= COLOR_PRECALC_LEN) {
		return ENC_SYM_EERROR;
	}

	color->precalc[pos] = new_color;

	return ENC_SYM_TRUE;
}

static lbm_value ext_clear(lbm_value *args, lbm_uint argn) {
	if ((argn != 1 && argn != 2) ||
			!lispif_disp_is_image_buffer(args[0]) ||
			(argn == 2 && !lbm_is_number(args[1]))) {
		return ENC_SYM_TERROR;
	}

	image_buffer_t *img = (image_buffer_t*)lbm_get_custom_value(args[0]);

	uint32_t color = 0;
	if (argn == 2) {
		color = lbm_dec_as_u32(args[1]);
	}

	image_buffer_clear(img, color);

	return ENC_SYM_TRUE;
}

static lbm_value ext_putpixel(lbm_value *args, lbm_uint argn) {
	img_args_t arg_dec = decode_args(args, argn, 3);

	if (!arg_dec.is_valid) {
		return ENC_SYM_TERROR;
	}

	putpixel(arg_dec.img,
			lbm_dec_as_i32(arg_dec.args[0]),
			lbm_dec_as_i32(arg_dec.args[1]),
			lbm_dec_as_i32(arg_dec.args[2]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_line(lbm_value *args, lbm_uint argn) {
	img_args_t arg_dec = decode_args(args, argn, 5);

	if (!arg_dec.is_valid) {
		return ENC_SYM_TERROR;
	}

	line(arg_dec.img,
			lbm_dec_as_i32(arg_dec.args[0]),
			lbm_dec_as_i32(arg_dec.args[1]),
			lbm_dec_as_i32(arg_dec.args[2]),
			lbm_dec_as_i32(arg_dec.args[3]),
			lbm_dec_as_i32(arg_dec.attr_thickness.args[0]),
			lbm_dec_as_i32(arg_dec.attr_dotted.args[0]),
			lbm_dec_as_i32(arg_dec.attr_dotted.args[1]),
			lbm_dec_as_i32(arg_dec.args[4]));

	return ENC_SYM_TRUE;
}

static lbm_value ext_circle(lbm_value *args, lbm_uint argn) {
	img_args_t arg_dec = decode_args(args, argn, 4);

	if (!arg_dec.is_valid) {
		return ENC_SYM_TERROR;
	}

	if (arg_dec.attr_filled.is_valid) {
		fill_circle(arg_dec.img,
				lbm_dec_as_i32(arg_dec.args[0]),
				lbm_dec_as_i32(arg_dec.args[1]),
				lbm_dec_as_i32(arg_dec.args[2]),
				lbm_dec_as_i32(arg_dec.args[3]));
	} if (arg_dec.attr_dotted.is_valid) {
		arc(arg_dec.img,
				lbm_dec_as_i32(arg_dec.args[0]),
				lbm_dec_as_i32(arg_dec.args[1]),
				lbm_dec_as_i32(arg_dec.args[2]),
				0, 359.9,
				lbm_dec_as_i32(arg_dec.attr_thickness.args[0]),
				false,
				lbm_dec_as_i32(arg_dec.attr_dotted.args[0]),
				lbm_dec_as_i32(arg_dec.attr_dotted.args[1]),
				false, false,
				lbm_dec_as_i32(arg_dec.args[3]));
	} else {
		circle(arg_dec.img,
				lbm_dec_as_i32(arg_dec.args[0]),
				lbm_dec_as_i32(arg_dec.args[1]),
				lbm_dec_as_i32(arg_dec.args[2]),
				lbm_dec_as_i32(arg_dec.attr_thickness.args[0]),
				lbm_dec_as_i32(arg_dec.args[3]));
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_arc(lbm_value *args, lbm_uint argn) {
	img_args_t arg_dec = decode_args(args, argn, 6);

	if (!arg_dec.is_valid) {
		return ENC_SYM_TERROR;
	}

	arc(arg_dec.img,
			lbm_dec_as_i32(arg_dec.args[0]),
			lbm_dec_as_i32(arg_dec.args[1]),
			lbm_dec_as_i32(arg_dec.args[2]),
			lbm_dec_as_float(arg_dec.args[3]),
			lbm_dec_as_float(arg_dec.args[4]),
			lbm_dec_as_i32(arg_dec.attr_thickness.args[0]),
			arg_dec.attr_filled.is_valid,
			lbm_dec_as_i32(arg_dec.attr_dotted.args[0]),
			lbm_dec_as_i32(arg_dec.attr_dotted.args[1]),
			false, false,
			lbm_dec_as_i32(arg_dec.args[5]));

	return ENC_SYM_TRUE;
}

static lbm_value ext_circle_sector(lbm_value *args, lbm_uint argn) {
	img_args_t arg_dec = decode_args(args, argn, 6);

	if (!arg_dec.is_valid) {
		return ENC_SYM_TERROR;
	}

	arc(arg_dec.img,
			lbm_dec_as_i32(arg_dec.args[0]),
			lbm_dec_as_i32(arg_dec.args[1]),
			lbm_dec_as_i32(arg_dec.args[2]),
			lbm_dec_as_float(arg_dec.args[3]),
			lbm_dec_as_float(arg_dec.args[4]),
			lbm_dec_as_i32(arg_dec.attr_thickness.args[0]),
			arg_dec.attr_filled.is_valid,
			lbm_dec_as_i32(arg_dec.attr_dotted.args[0]),
			lbm_dec_as_i32(arg_dec.attr_dotted.args[1]),
			true, false,
			lbm_dec_as_i32(arg_dec.args[5]));

	return ENC_SYM_TRUE;
}

static lbm_value ext_circle_segment(lbm_value *args, lbm_uint argn) {
	img_args_t arg_dec = decode_args(args, argn, 6);

	if (!arg_dec.is_valid) {
		return ENC_SYM_TERROR;
	}

	arc(arg_dec.img,
			lbm_dec_as_i32(arg_dec.args[0]),
			lbm_dec_as_i32(arg_dec.args[1]),
			lbm_dec_as_i32(arg_dec.args[2]),
			lbm_dec_as_float(arg_dec.args[3]),
			lbm_dec_as_float(arg_dec.args[4]),
			lbm_dec_as_i32(arg_dec.attr_thickness.args[0]),
			arg_dec.attr_filled.is_valid,
			lbm_dec_as_i32(arg_dec.attr_dotted.args[0]),
			lbm_dec_as_i32(arg_dec.attr_dotted.args[1]),
			false, true,
			lbm_dec_as_i32(arg_dec.args[5]));

	return ENC_SYM_TRUE;
}

static lbm_value ext_rectangle(lbm_value *args, lbm_uint argn) {
	img_args_t arg_dec = decode_args(args, argn, 5);

	if (!arg_dec.is_valid) {
		return ENC_SYM_TERROR;
	}

	image_buffer_t *img = arg_dec.img;
	int x = lbm_dec_as_i32(arg_dec.args[0]);
	int y = lbm_dec_as_i32(arg_dec.args[1]);
	int width = lbm_dec_as_i32(arg_dec.args[2]);
	int height = lbm_dec_as_i32(arg_dec.args[3]);
	int rad = lbm_dec_as_i32(arg_dec.attr_rounded.args[0]);
	int thickness = lbm_dec_as_i32(arg_dec.attr_thickness.args[0]);
	uint32_t color = lbm_dec_as_i32(arg_dec.args[4]);
	int dot1 = lbm_dec_as_i32(arg_dec.attr_dotted.args[0]);
	int dot2 = lbm_dec_as_i32(arg_dec.attr_dotted.args[1]);

	if (arg_dec.attr_rounded.is_valid) {
		if (arg_dec.attr_filled.is_valid) {
			rectangle(img, x + rad, y, width - 2 * rad, rad, 1, 1, 0, 0, color);
			rectangle(img, x + rad, y + height - rad, width - 2 * rad, rad, 1, 1, 0, 0, color);
			rectangle(img, x, y + rad, width, height - 2 * rad, 1, 1, 0, 0, color);
			fill_circle(img, x + rad, y + rad, rad, color);
			fill_circle(img, x + rad, y + height - rad, rad, color);
			fill_circle(img, x + width - rad, y + rad, rad, color);
			fill_circle(img, x + width - rad, y + height - rad, rad, color);
		} else {
			line(img, x + rad, y, x + width - rad, y, thickness, dot1, dot2, color);
			arc(img, x + rad, y + rad, rad, 180, 270, thickness, false, dot1, dot2, false, false, color);
			line(img, x + rad, y + height, x + width - rad, y + height, thickness, dot1, dot2, color);
			arc(img, x + rad, y + height - rad, rad, 90, 180, thickness, false, dot1, dot2, false, false, color);
			line(img, x, y + rad, x, y + height - rad, thickness, dot1, dot2, color);
			arc(img, x + width - rad, y + height - rad, rad, 0, 90, thickness, false, dot1, dot2, false, false, color);
			line(img, x + width, y + rad, x + width, y + height - rad, thickness, dot1, dot2, color);
			arc(img, x + width - rad, y + rad, rad, 270, 0, thickness, false, dot1, dot2, false, false, color);
		}
	} else {
		rectangle(img,
				x, y,
				width, height,
				arg_dec.attr_filled.is_valid,
				thickness,
				dot1, dot2,
				color);
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_triangle(lbm_value *args, lbm_uint argn) {
	img_args_t arg_dec = decode_args(args, argn, 7);

	if (!arg_dec.is_valid) {
		return ENC_SYM_TERROR;
	}

	image_buffer_t *img = arg_dec.img;
	int x0 = lbm_dec_as_i32(arg_dec.args[0]);
	int y0 = lbm_dec_as_i32(arg_dec.args[1]);
	int x1 = lbm_dec_as_i32(arg_dec.args[2]);
	int y1 = lbm_dec_as_i32(arg_dec.args[3]);
	int x2 = lbm_dec_as_i32(arg_dec.args[4]);
	int y2 = lbm_dec_as_i32(arg_dec.args[5]);
	int thickness = lbm_dec_as_i32(arg_dec.attr_thickness.args[0]);
	int dot1 = lbm_dec_as_i32(arg_dec.attr_dotted.args[0]);
	int dot2 = lbm_dec_as_i32(arg_dec.attr_dotted.args[1]);
	uint32_t color = lbm_dec_as_i32(arg_dec.args[6]);

	if (arg_dec.attr_filled.is_valid) {
		fill_triangle(img, x0, y0, x1, y1, x2, y2, color);
	} else {
		line(img, x0, y0, x1, y1, thickness, dot1, dot2, color);
		line(img, x1, y1, x2, y2, thickness, dot1, dot2, color);
		line(img, x2, y2, x0, y0, thickness, dot1, dot2, color);
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_text(lbm_value *args, lbm_uint argn) {
	if (argn != 7) {
		return ENC_SYM_TERROR;
	}

	int x = lbm_dec_as_u32(args[1]);
	int y = lbm_dec_as_u32(args[2]);
	int fg = lbm_dec_as_i32(args[3]);
	int bg = lbm_dec_as_i32(args[4]);

	if (!lispif_disp_is_image_buffer(args[0])) return ENC_SYM_TERROR;
	image_buffer_t *img = (image_buffer_t*)lbm_get_custom_value(args[0]);

	lbm_array_header_t *font = 0;
	if (lbm_type_of(args[5]) == LBM_TYPE_ARRAY) {
		font = (lbm_array_header_t *)lbm_car(args[5]);
		if (font->elt_type != LBM_TYPE_BYTE) {
			font = 0;
		}
	}

	char *txt = lbm_dec_str(args[6]);

	if (!font || !txt || font->size < (4 + 5 * 5 * 10)) {
		return ENC_SYM_TERROR;
	}

	uint8_t *font_data = (uint8_t*)font->data;
	uint8_t w = font_data[0];

	int ind = 0;
	while (txt[ind] != 0) {
		img_putc(img, x + ind * w, y, fg, bg, font_data, txt[ind]);
		ind++;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_blit(lbm_value *args, lbm_uint argn) {
	img_args_t arg_dec = decode_args(args + 1, argn - 1, 3);

	if (!arg_dec.is_valid) {
		return ENC_SYM_TERROR;
	}

	if (!lispif_disp_is_image_buffer(args[0])) {
		return ENC_SYM_TERROR;
	}

	image_buffer_t *dest = (image_buffer_t*)lbm_get_custom_value(args[0]);

	float scale = 1.0;
	if (arg_dec.attr_scale.is_valid) {
		scale = lbm_dec_as_float(arg_dec.attr_scale.args[0]);
	}

	blit_rot_scale(
			dest,
			arg_dec.img,
			lbm_dec_as_i32(arg_dec.args[0]),
			lbm_dec_as_i32(arg_dec.args[1]),
			lbm_dec_as_float(arg_dec.attr_rotate.args[0]),
			lbm_dec_as_float(arg_dec.attr_rotate.args[1]),
			lbm_dec_as_float(arg_dec.attr_rotate.args[2]),
			scale,
			lbm_dec_as_i32(arg_dec.args[2]));

	return ENC_SYM_TRUE;
}

// Display Drivers

static bool gpio_is_valid(int pin) {
	switch (pin) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
	case 18:
	case 19:
	case 20:
	case 21:
		return true;

	default:
		return false;
	}
}

static bool(* volatile disp_render_image)(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) = 0;
static void(* volatile disp_clear)(uint32_t color) = 0;
static void(* volatile disp_reset)(void) = 0;

static char *msg_invalid_gpio = "Invalid GPIO";
static char *msg_invalid_clk_speed = "Invalid clock speed";
static char *msg_not_supported = "Command not supported or display driver not initialized";

static lbm_value ext_disp_reset(lbm_value *args, lbm_uint argn) {
	(void) args;
	(void) argn;

	if (disp_reset == NULL) {
		lbm_set_error_reason(msg_not_supported);
		return ENC_SYM_EERROR;
	}

	disp_reset();

	return ENC_SYM_TRUE;
}

static lbm_value ext_disp_clear(lbm_value *args, lbm_uint argn) {
	if (disp_clear == NULL) {
		lbm_set_error_reason(msg_not_supported);
		return ENC_SYM_EERROR;
	}

	if (argn > 1) {
		return ENC_SYM_TERROR;
	}

	uint32_t clear_color = 0;

	if (argn == 1) {
		if (!lbm_is_number(args[0])) {
			return ENC_SYM_TERROR;
		}

		clear_color = lbm_dec_as_u32(args[0]);
	}

	disp_clear(clear_color);

	return ENC_SYM_TRUE;
}

static lbm_value ext_disp_render(lbm_value *args, lbm_uint argn) {
	if (disp_render_image == NULL) {
		lbm_set_error_reason(msg_not_supported);
		return ENC_SYM_EERROR;
	}

	if ((argn != 3 && argn != 4) ||
			!lispif_disp_is_image_buffer(args[0]) ||
			!lbm_is_number(args[1]) ||
			!lbm_is_number(args[2])) {
		return ENC_SYM_TERROR;
	}

	image_buffer_t *img = (image_buffer_t*)lbm_get_custom_value(args[0]);

	color_t colors[4];
	memset(colors, 0, sizeof(color_t) * 4);

	if (argn == 4 && lbm_is_list(args[3])) {
		int i = 0;
		lbm_value curr = args[3];
		while (lbm_is_cons(curr) && i < 4) {
			lbm_value arg = lbm_car(curr);

			if (lbm_is_number(arg)) {
				colors[i].color1 = lbm_dec_as_u32(arg);
			} else if (lispif_disp_is_color(arg)) {
				colors[i] = *((color_t*)lbm_get_custom_value(arg));
			} else {
				return ENC_SYM_TERROR;
			}

			curr = lbm_cdr(curr);
			i++;
		}
	}

	bool render_res = disp_render_image(img, lbm_dec_as_u32(args[1]), lbm_dec_as_u32(args[2]), colors);

	if (!render_res) {
		lbm_set_error_reason("Could not render image. Check if the format and location is compatible with the display.");
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_disp_load_sh8501b(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(5);

	int gpio_sd0, gpio_clk, gpio_cs, gpio_reset;
	gpio_sd0 = lbm_dec_as_i32(args[0]);
	gpio_clk = lbm_dec_as_i32(args[1]);
	gpio_cs = lbm_dec_as_i32(args[2]);
	gpio_reset = lbm_dec_as_i32(args[3]);

	if (!gpio_is_valid(gpio_sd0) ||
			!gpio_is_valid(gpio_clk) ||
			!gpio_is_valid(gpio_cs) ||
			!gpio_is_valid(gpio_reset)) {
		lbm_set_error_reason(msg_invalid_gpio);
		return ENC_SYM_EERROR;
	}

	int spi_mhz = lbm_dec_as_i32(args[4]);

	if (spi_mhz > 40) {
		lbm_set_error_reason(msg_invalid_clk_speed);
		return ENC_SYM_EERROR;
	}

	disp_sh8501b_init(gpio_sd0, gpio_clk, gpio_cs, gpio_reset, spi_mhz);

	disp_render_image = disp_sh8501b_render_image;
	disp_clear = disp_sh8501b_clear;
	disp_reset = disp_sh8501b_reset;

	return ENC_SYM_TRUE;
}

static lbm_value ext_disp_load_ili9341(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(6);

	int gpio_sd0, gpio_clk, gpio_cs, gpio_reset, gpio_dc;
	gpio_sd0 = lbm_dec_as_i32(args[0]);
	gpio_clk = lbm_dec_as_i32(args[1]);
	gpio_cs = lbm_dec_as_i32(args[2]);
	gpio_reset = lbm_dec_as_i32(args[3]);
	gpio_dc = lbm_dec_as_i32(args[4]);

	if (!gpio_is_valid(gpio_sd0) ||
			!gpio_is_valid(gpio_clk) ||
			!gpio_is_valid(gpio_cs) ||
			!gpio_is_valid(gpio_reset) ||
			!gpio_is_valid(gpio_dc)) {
		lbm_set_error_reason(msg_invalid_gpio);
		return ENC_SYM_EERROR;
	}

	int spi_mhz = lbm_dec_as_i32(args[5]);

	if (spi_mhz > 40) {
		lbm_set_error_reason(msg_invalid_clk_speed);
		return ENC_SYM_EERROR;
	}

	disp_ili9341_init(gpio_sd0, gpio_clk, gpio_cs, gpio_reset, gpio_dc, spi_mhz);

	disp_render_image = disp_ili9341_render_image;
	disp_clear = disp_ili9341_clear;
	disp_reset = disp_ili9341_reset;

	return ENC_SYM_TRUE;
}

static lbm_value ext_disp_load_ssd1306(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(3);

	int gpio_sda = lbm_dec_as_i32(args[0]);
	int gpio_scl = lbm_dec_as_i32(args[1]);
	uint32_t clk_speed = lbm_dec_as_u32(args[2]);

	if (!gpio_is_valid(gpio_sda) ||
			!gpio_is_valid(gpio_scl)) {
		lbm_set_error_reason(msg_invalid_gpio);
		return ENC_SYM_EERROR;
	}

	if (clk_speed > 8000000) {
		lbm_set_error_reason(msg_invalid_clk_speed);
		return ENC_SYM_EERROR;
	}

	disp_ssd1306_init(gpio_sda, gpio_scl, clk_speed);

	disp_render_image = disp_ssd1306_render_image;
	disp_clear = disp_ssd1306_clear;
	disp_reset = disp_ssd1306_reset;

	return ENC_SYM_TRUE;
}


// Jpg decoder

typedef struct {
	uint8_t *data;
	int pos;
	int size;
	int ofs_x;
	int ofs_y;
} jpg_bufdef;

size_t jpg_input_func (JDEC* jd, uint8_t* buff, size_t ndata) {
	jpg_bufdef *dev = (jpg_bufdef*)jd->device;

	if (ndata > (dev->size - dev->pos)) {
		ndata = (dev->size - dev->pos);
	}

	if (buff) {
		memcpy(buff, dev->data + dev->pos, ndata);
	}
	dev->pos += ndata;
	return ndata;
}

int jpg_output_func (	/* 1:Ok, 0:Aborted */
	JDEC* jd,		/* Decompression object */
	void* bitmap,	/* Bitmap data to be output */
	JRECT* rect		/* Rectangular region to output */
) {
	jpg_bufdef *dev = (jpg_bufdef*)jd->device;

	image_buffer_t img;
	memset(&img, 0, sizeof(img));

	img.fmt = rgb888;
	img.width = rect->right - rect->left + 1;
	img.height = rect->bottom - rect->top + 1;
	img.data = bitmap;

	disp_render_image(&img, rect->left + dev->ofs_x, rect->top + dev->ofs_y, 0);

	return 1;
}

static lbm_value ext_disp_render_jpg(lbm_value *args, lbm_uint argn) {
	if (disp_render_image == NULL) {
		lbm_set_error_reason(msg_not_supported);
		return ENC_SYM_EERROR;
	}

	if (argn != 3 ||
			!lbm_is_byte_array(args[0]) ||
			!lbm_is_number(args[1]) ||
			!lbm_is_number(args[2])) {
		return ENC_SYM_TERROR;
	}

	JDEC jd;
	void *jdwork;
	const size_t sz_work = 4096;

	jdwork = lbm_malloc(sz_work);
	if (!jdwork) {
		return ENC_SYM_MERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);

	jpg_bufdef iodev;
	iodev.data = (uint8_t*)(array->data);
	iodev.size = array->size;
	iodev.pos = 0;
	iodev.ofs_x = lbm_dec_as_i32(args[1]);
	iodev.ofs_y = lbm_dec_as_i32(args[2]);

	jd_prepare(&jd, jpg_input_func, jdwork, sz_work, &iodev);
	jd_decomp(&jd, jpg_output_func, 0);

	lbm_free(jdwork);

	return ENC_SYM_TRUE;
}

void lispif_load_disp_extensions(void) {
	register_symbols();

	disp_render_image = NULL;
	disp_clear = NULL;
	disp_reset = NULL;

	lbm_add_extension("img-buffer", ext_image_buffer);
	lbm_add_extension("img-buffer-from-bin", ext_image_buffer_from_bin);
	lbm_add_extension("img-color", ext_color);
	lbm_add_extension("img-color-setpre", ext_color_setpre);
	lbm_add_extension("img-dims", ext_image_dims);
	lbm_add_extension("img-setpix", ext_putpixel);
	lbm_add_extension("img-line", ext_line);
	lbm_add_extension("img-text", ext_text);
	lbm_add_extension("img-clear", ext_clear);
	lbm_add_extension("img-circle", ext_circle);
	lbm_add_extension("img-arc", ext_arc);
	lbm_add_extension("img-circle-sector", ext_circle_sector);
	lbm_add_extension("img-circle-segment", ext_circle_segment);
	lbm_add_extension("img-rectangle", ext_rectangle);
	lbm_add_extension("img-triangle", ext_triangle);
	lbm_add_extension("img-blit", ext_blit);

	lbm_add_extension("disp-load-sh8501b", ext_disp_load_sh8501b);
	lbm_add_extension("disp-load-ili9341", ext_disp_load_ili9341);
	lbm_add_extension("disp-load-ssd1306", ext_disp_load_ssd1306);
	lbm_add_extension("disp-reset", ext_disp_reset);
	lbm_add_extension("disp-clear", ext_disp_clear);
	lbm_add_extension("disp-render", ext_disp_render);
	lbm_add_extension("disp-render-jpg", ext_disp_render_jpg);
}
