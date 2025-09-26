/*
	Copyright 2024 Benjamin Vedder	benjamin@vedder.se

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
#include <stdint.h>   // for uint8_t, uint32_t, etc.
#include <stdbool.h>  // for bool, true, false
#include <stddef.h>   // for size_t

#include "lispif_rgbled_extensions.h"
#include "lispif.h"
#include "lispbm.h"
#include "utils.h"
#include "commands.h"

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "soc/rmt_periph.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_periph.h"
#include "hal/gpio_hal.h"
#include "driver/gpio.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

typedef struct {
	rmt_encoder_t base;
	rmt_encoder_t *bytes_encoder;
	rmt_encoder_t *copy_encoder;
	int state;
	rmt_symbol_word_t reset_code;
} rmt_led_strip_encoder_t;

static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static unsigned int led_type_driver = 0;
static int led_pin_driver = -1;

static const uint8_t gamma_table[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3,
		4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10,
		10, 11, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 16, 16, 17, 17, 18,
		18, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 29,
		29, 30, 31, 31, 32, 33, 34, 34, 35, 36, 37, 37, 38, 39, 40, 40, 41, 42,
		43, 44, 45, 46, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 58,
		59, 60, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 78,
		79, 80, 81, 83, 84, 85, 86, 87, 89, 90, 91, 92, 94, 95, 96, 98, 99, 100,
		102, 103, 104, 106, 107, 109, 110, 111, 113, 114, 116, 117, 119, 120,
		121, 123, 124, 126, 127, 129, 131, 132, 134, 135, 137, 138, 140, 142,
		143, 145, 146, 148, 150, 151, 153, 155, 157, 158, 160, 162, 163, 165,
		167, 169, 170, 172, 174, 176, 178, 180, 181, 183, 185, 187, 189, 191,
		193, 195, 197, 198, 200, 202, 204, 206, 208, 210, 212, 214, 216, 218,
		220, 223, 225, 227, 229, 231, 233, 235, 237, 239, 242, 244, 246, 248,
		250, 253, 255 };

static rmt_transmit_config_t tx_config = {
		.loop_count = 0, // no transfer loop
};

static size_t rmt_encode_led_strip(
		rmt_encoder_t *encoder, rmt_channel_handle_t channel,
		const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state) {
	rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
	rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
	rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
	rmt_encode_state_t session_state = 0;
	rmt_encode_state_t state = 0;
	size_t encoded_symbols = 0;
	switch (led_encoder->state) {
	case 0: // send RGB data
		encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
		if (session_state & RMT_ENCODING_COMPLETE) {
			led_encoder->state = 1; // switch to next state when current encoding session finished
		}
		if (session_state & RMT_ENCODING_MEM_FULL) {
			state |= RMT_ENCODING_MEM_FULL;
			goto out; // yield if there's no free space for encoding artifacts
		}
		// fall-through
		//no break
	case 1: // send reset code
		encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
				sizeof(led_encoder->reset_code), &session_state);
		if (session_state & RMT_ENCODING_COMPLETE) {
			led_encoder->state = 0; // back to the initial encoding session
			state |= RMT_ENCODING_COMPLETE;
		}
		if (session_state & RMT_ENCODING_MEM_FULL) {
			state |= RMT_ENCODING_MEM_FULL;
			goto out; // yield if there's no free space for encoding artifacts
		}
	}
	out:
	*ret_state = state;
	return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder) {
	rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
	rmt_del_encoder(led_encoder->bytes_encoder);
	rmt_del_encoder(led_encoder->copy_encoder);
	free(led_encoder);
	return ESP_OK;
}

static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder) {
	rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
	rmt_encoder_reset(led_encoder->bytes_encoder);
	rmt_encoder_reset(led_encoder->copy_encoder);
	led_encoder->state = 0;
	return ESP_OK;
}

esp_err_t rmt_new_led_strip_encoder(rmt_encoder_handle_t *ret_encoder) {
	rmt_led_strip_encoder_t *led_encoder = NULL;
	led_encoder = calloc(1, sizeof(rmt_led_strip_encoder_t));
	led_encoder->base.encode = rmt_encode_led_strip;
	led_encoder->base.del = rmt_del_led_strip_encoder;
	led_encoder->base.reset = rmt_led_strip_encoder_reset;

	// different led strip might have its own timing requirements, following parameter is for WS2812
	rmt_bytes_encoder_config_t bytes_encoder_config = {
			.bit0 = {
					.level0 = 1,
					.duration0 = 0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T0H=0.3us
					.level1 = 0,
					.duration1 = 0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T0L=0.9us
			},
			.bit1 = {
					.level0 = 1,
					.duration0 = 0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T1H=0.9us
					.level1 = 0,
					.duration1 = 0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T1L=0.3us
			},
			.flags.msb_first = 1 // WS2812 transfer bit order: G7...G0R7...R0B7...B0
	};

	rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder);
	rmt_copy_encoder_config_t copy_encoder_config = {};
	rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder);

	uint32_t reset_ticks = RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2; // reset code duration defaults to 50us
	led_encoder->reset_code = (rmt_symbol_word_t) {
		.level0 = 0,
				.duration0 = reset_ticks,
				.level1 = 0,
				.duration1 = reset_ticks,
	};

	*ret_encoder = &led_encoder->base;
	return ESP_OK;
}

void rgbled_deinit() {
	if (led_chan != NULL) {
		rmt_tx_wait_all_done(led_chan, 100);
		rmt_disable(led_chan);
		rmt_del_channel(led_chan);
		led_chan = NULL;
	}

	if (led_encoder != NULL) {
		rmt_del_encoder(led_encoder);
		led_encoder = NULL;
	}

	if (led_pin_driver >= 0) {
		gpio_reset_pin(led_pin_driver);
		led_pin_driver = -1;
	}
}

static lbm_value ext_rgbled_deinit(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	rgbled_deinit();

	return ENC_SYM_TRUE;
}

bool rgbled_init(int pin) {
	rgbled_deinit();

	led_pin_driver = pin;

	rmt_tx_channel_config_t tx_chan_config = {
			.clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
			.gpio_num = pin,
			.mem_block_symbols = 64, // increase the block size can make the LED less flickering
			.resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
			.trans_queue_depth = 4, // set the number of transactions that can be pending in the background
	};
	rmt_new_tx_channel(&tx_chan_config, &led_chan);

	rmt_new_led_strip_encoder(&led_encoder);
	rmt_enable(led_chan);

	return true;
}

static lbm_value ext_rgbled_init(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	if (argn != 1 && argn != 2) {
		lbm_set_error_reason((char*)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}

	int pin = lbm_dec_as_i32(args[0]);
	if (!utils_gpio_is_valid(pin)) {
		lbm_set_error_reason(string_pin_invalid);
		return ENC_SYM_TERROR;
	}

	unsigned int type_led = 0;
	if (argn >= 2) {
		type_led = lbm_dec_as_u32(args[1]);
		if (type_led > 4) {
			lbm_set_error_reason("Invalid LED type");
			return ENC_SYM_TERROR;
		}
	}
	rgbled_init(pin);

	return ENC_SYM_TRUE;
}

static lbm_value ext_rgbled_color_buffer(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	unsigned int num_led = lbm_dec_as_u32(args[0]);

	uint8_t type_led = 0;
	if (argn >= 2) {
		type_led = lbm_dec_as_u32(args[1]);
		if (type_led > 4) {
			lbm_set_error_reason("Invalid LED type");
			return ENC_SYM_TERROR;
		}
	}

	uint8_t gamma_corr = 0;
	if (argn >= 3) {
		gamma_corr = lbm_dec_as_u32(args[2]);
		if (gamma_corr > 1) {
			gamma_corr = 1;
		}
	}

	int led_colors = 3;
	if (type_led >= 2) {
		led_colors = 4;
	}

	lbm_value res;
	if (lbm_create_array(&res, num_led * led_colors + 1)) {
		lbm_array_header_t *arr = (lbm_array_header_t*)lbm_car(res);
		uint8_t *data = (uint8_t*)arr->data;
		memset(data, 0, arr->size);
		data[0] = type_led | (gamma_corr << 4);
		return res;
	} else {
		return ENC_SYM_MERROR;
	}
}

static lbm_value ext_rgbled_color(lbm_value *args, lbm_uint argn) {
	if ((argn != 3 && argn != 4) || !lbm_is_array_r(args[0]) ||
			!lbm_is_number(args[1]) || (!lbm_is_number(args[2]) && !lbm_is_list(args[2]))) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);
	uint8_t *led_data = (uint8_t*)array->data;
	uint8_t *led_pixels = led_data + 1;
	int led_data_len = array->size - 1;

	char *invalid_arr_msg = "Invalid LED array";

	uint8_t type_led = led_data[0] & 0x0F;
	if (type_led > 4) {
		lbm_set_error_reason(invalid_arr_msg);
		return ENC_SYM_TERROR;
	}

	uint8_t gamma_corr = led_data[0] >> 4;

	int led_colors = 3;
	if (type_led >= 2) {
		led_colors = 4;
	}

	if (led_data_len % led_colors != 0) {
		lbm_set_error_reason(invalid_arr_msg);
		return ENC_SYM_TERROR;
	}

	int led_num = led_data_len / led_colors;
	int led = lbm_dec_as_u32(args[1]);

	int curr = args[2];
	bool number = lbm_is_number(curr);

	float brightness = -1.0;
	if (argn == 4) {
		if (!lbm_is_number(args[3])) {
			lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
			return ENC_SYM_TERROR;
		}

		brightness = lbm_dec_as_float(args[3]);
		utils_truncate_number(&brightness, 0.0, 1.0);
	}

	while (lbm_is_cons(curr) || number) {
		lbm_value arg;

		if (number) {
			arg = curr;
		} else {
			arg = lbm_car(curr);
		}

		if (led >= led_num) {
			break;
		}

		if (lbm_is_number(arg)) {
			uint32_t color = lbm_dec_as_u32(arg);

			uint8_t w = (color >> 24) & 0xFF;
			uint8_t r = (color >> 16) & 0xFF;
			uint8_t g = (color >> 8) & 0xFF;
			uint8_t b = color & 0xFF;

			if (brightness >= 0.0) {
				w = (uint8_t)roundf((float)w * brightness);
				r = (uint8_t)roundf((float)r * brightness);
				g = (uint8_t)roundf((float)g * brightness);
				b = (uint8_t)roundf((float)b * brightness);
			}

			if (gamma_corr) {
				w = gamma_table[w];
				r = gamma_table[r];
				g = gamma_table[g];
				b = gamma_table[b];
			}

			switch (type_led) {
			case 0: // GRB
				led_pixels[led * 3 + 0] = g;
				led_pixels[led * 3 + 1] = r;
				led_pixels[led * 3 + 2] = b;
				break;

			case 1: // RGB
				led_pixels[led * 3 + 0] = r;
				led_pixels[led * 3 + 1] = g;
				led_pixels[led * 3 + 2] = b;
				break;

			case 2: // GRBW
				led_pixels[led * 4 + 0] = g;
				led_pixels[led * 4 + 1] = r;
				led_pixels[led * 4 + 2] = b;
				led_pixels[led * 4 + 3] = w;
				break;

			case 3: // RGBW
				led_pixels[led * 4 + 0] = r;
				led_pixels[led * 4 + 1] = g;
				led_pixels[led * 4 + 2] = b;
				led_pixels[led * 4 + 3] = w;
				break;

			case 4: // WRGB
				led_pixels[led * 4 + 0] = w;
				led_pixels[led * 4 + 1] = r;
				led_pixels[led * 4 + 2] = g;
				led_pixels[led * 4 + 3] = b;
				break;

			default:
				break;
			}
		} else {
			return ENC_SYM_EERROR;
		}

		if (number) {
			break;
		}

		led++;
		curr = lbm_cdr(curr);
	}

	return ENC_SYM_TRUE;
}

void rgbled_update(uint8_t * data, size_t size) {
    if (size < 1) return;
    rmt_transmit(led_chan, led_encoder, data, size, &tx_config);
}

static lbm_value ext_rgbled_update(lbm_value *args, lbm_uint argn) {
	if (led_encoder == NULL || led_chan == NULL) {
		lbm_set_error_reason("Please run rgbled-init first");
		if (led_encoder == NULL) {
			commands_printf_lisp("led_encoder null");
		}

		if (led_chan == NULL) {
			commands_printf_lisp("led_chan null");
		}

		return ENC_SYM_EERROR;
	}

	if (argn != 1 || !lbm_is_array_r(args[0])) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);

	rgbled_update((uint8_t*)array->data + 1, array->size - 1);

	return ENC_SYM_TRUE;
}

void lispif_load_rgbled_extensions(void) {
	lbm_add_extension("rgbled-init", ext_rgbled_init);
	lbm_add_extension("rgbled-deinit", ext_rgbled_deinit);
	lbm_add_extension("rgbled-buffer", ext_rgbled_color_buffer);
	lbm_add_extension("rgbled-color", ext_rgbled_color);
	lbm_add_extension("rgbled-update", ext_rgbled_update);
}
