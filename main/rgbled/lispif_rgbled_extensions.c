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

static lbm_value ext_rgbled_deinit(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

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

	return ENC_SYM_TRUE;
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
		if (type_led >= 4) {
			lbm_set_error_reason("Invalid LED type");
			return ENC_SYM_TERROR;
		}
	}

	ext_rgbled_deinit(0, 0);

	led_type_driver = type_led;
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

	return ENC_SYM_TRUE;
}

static lbm_value ext_rgbled_color_buffer(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	unsigned int num_led = lbm_dec_as_u32(args[0]);

	unsigned int type_led = 0;
	if (argn >= 2) {
		type_led = lbm_dec_as_u32(args[1]);
		if (type_led >= 4) {
			lbm_set_error_reason("Invalid LED type");
			return ENC_SYM_TERROR;
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
		data[0] = type_led;
		return res;
	} else {
		return ENC_SYM_MERROR;
	}
}

static lbm_value ext_rgbled_color(lbm_value *args, lbm_uint argn) {
	if (argn != 3 || !lbm_is_array_r(args[0]) ||
			!lbm_is_number(args[1]) || !lbm_is_number(args[2])) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);
	uint8_t *led_data = (uint8_t*)array->data;
	uint8_t *led_pixels = led_data + 1;
	int led_data_len = array->size - 1;

	char *invalid_arr_msg = "Invalid LED array";

	unsigned int type_led = led_data[0];
	if (type_led >= 4) {
		lbm_set_error_reason(invalid_arr_msg);
		return ENC_SYM_TERROR;
	}

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
	if (led >= led_num) {
		lbm_set_error_reason("Invalid LED number");
		return ENC_SYM_TERROR;
	}

	uint32_t color = lbm_dec_as_u32(args[2]);

	uint8_t w = (color >> 24) & 0xFF;
	uint8_t r = (color >> 16) & 0xFF;
	uint8_t g = (color >> 8) & 0xFF;
	uint8_t b = color & 0xFF;

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

	default:
		break;
	}

	return ENC_SYM_TRUE;
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

	rmt_transmit(led_chan, led_encoder, (uint8_t*)array->data + 1, array->size - 1, &tx_config);

	return ENC_SYM_TRUE;
}

void lispif_load_rgbled_extensions(void) {
	lbm_add_extension("rgbled-init", ext_rgbled_init);
	lbm_add_extension("rgbled-deinit", ext_rgbled_deinit);
	lbm_add_extension("rgbled-color-buffer", ext_rgbled_color_buffer);
	lbm_add_extension("rgbled-color", ext_rgbled_color);
	lbm_add_extension("rgbled-update", ext_rgbled_update);
}
