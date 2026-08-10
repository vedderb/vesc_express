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
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "lispif_rgbled_extensions.h"
#include "lispif.h"
#include "lispbm.h"
#include "utils.h"
#include "commands.h"

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_private/rmt.h"
#include "hal/rmt_periph.h"
#include "soc/soc_caps.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_periph.h"
#include "hal/gpio_hal.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

#define LED_STRIP_MAX 8   // distinct pins tracked at once
#define LED_TIMING_MAX 5  // wire timing presets, one encoder each

typedef struct {
	rmt_encoder_t base;
	rmt_encoder_t *bytes_encoder;
	rmt_encoder_t *copy_encoder;
	int state;
	rmt_symbol_word_t reset_code;
} rmt_led_strip_encoder_t;

typedef struct {
	int pin;                      // -1 = free entry
	unsigned int timing;
} led_strip_t;

static led_strip_t led_strip[LED_STRIP_MAX];
static rmt_channel_handle_t led_chan = NULL;          // the one TX channel
static rmt_encoder_handle_t led_enc[LED_TIMING_MAX];  // built on demand
static int led_routed_pin = -1;   // pin the channel's output is wired to
static int led_tx_sig = -1;       // the channel's TX signal, -1 if undiscovered
static int led_lisp_pin = -1; // pin targeted by the arg-less lisp rgbled-update
static bool led_state_init_done = false;
static const char *led_init_err = "LED strip init failed";

static SemaphoreHandle_t led_mtx = NULL;
static StaticSemaphore_t led_mtx_buf;

static void led_lock(void) {
	if (led_mtx != NULL) {
		xSemaphoreTakeRecursive(led_mtx, portMAX_DELAY);
	}
}

static void led_unlock(void) {
	if (led_mtx != NULL) {
		xSemaphoreGiveRecursive(led_mtx);
	}
}

static void rgbled_state_init_once(void) {
	if (led_state_init_done) {
		return;
	}
	for (int i = 0; i < LED_TIMING_MAX; i++) {
		led_enc[i] = NULL;
	}
	for (int i = 0; i < LED_STRIP_MAX; i++) {
		led_strip[i].pin = -1;
	}
	led_chan = NULL;
	led_routed_pin = -1;
	led_tx_sig = -1;
	led_state_init_done = true;
}

static float led_t0h;
static float led_t0l;
static float led_t1h;
static float led_t1l;
static uint32_t led_reset_us;

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
	rmt_led_strip_encoder_t *led_encoder = calloc(1, sizeof(rmt_led_strip_encoder_t));
	if (led_encoder == NULL) {
		return ESP_ERR_NO_MEM;
	}
	led_encoder->base.encode = rmt_encode_led_strip;
	led_encoder->base.del = rmt_del_led_strip_encoder;
	led_encoder->base.reset = rmt_led_strip_encoder_reset;

	rmt_bytes_encoder_config_t bytes_encoder_config = {
			.bit0 = {
					.level0 = 1,
					.duration0 = led_t0h * RMT_LED_STRIP_RESOLUTION_HZ / 1000000,
					.level1 = 0,
					.duration1 = led_t0l * RMT_LED_STRIP_RESOLUTION_HZ / 1000000,
			},
			.bit1 = {
					.level0 = 1,
					.duration0 = led_t1h * RMT_LED_STRIP_RESOLUTION_HZ / 1000000,
					.level1 = 0,
					.duration1 = led_t1l * RMT_LED_STRIP_RESOLUTION_HZ / 1000000,
			},
			.flags.msb_first = 1 // WS2812 transfer bit order: G7...G0R7...R0B7...B0
	};

	rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder);
	rmt_copy_encoder_config_t copy_encoder_config = {};
	rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder);

	uint32_t reset_ticks = RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * led_reset_us / 2; // configurable reset duration
	led_encoder->reset_code = (rmt_symbol_word_t) {
		.level0 = 0,
				.duration0 = reset_ticks,
				.level1 = 0,
				.duration1 = reset_ticks,
	};

	*ret_encoder = &led_encoder->base;
	return ESP_OK;
}

// Detach from the RMT signal and hold low. Addressable strips idle low, and a
// pin left on the signal would mirror whatever the next strip is sent.
static void led_pin_idle(int pin) {
	esp_rom_gpio_connect_out_signal((uint32_t)pin, SIG_GPIO_OUT_IDX, false, false);
	gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
	gpio_set_level((gpio_num_t)pin, 0);
}

static void led_find_tx_sig(int pin) {
	(void)pin;
	led_tx_sig = -1;
	int channel_id;
	if (rmt_get_channel_id(led_chan, &channel_id) == ESP_OK) {
		led_tx_sig = soc_rmt_signals[0].channels[channel_id].tx_sig;
	}
}

// Timing for the next encoder built.
static void led_timing_apply(unsigned int timing) {
	switch (timing) {
	case 1: // WS2812B
		led_t0h = 0.40; led_t0l = 0.85; led_t1h = 0.80; led_t1l = 0.45; led_reset_us = 300;
		break;
	case 2: // WS2815
		led_t0h = 0.30; led_t0l = 0.90; led_t1h = 0.90; led_t1l = 0.30; led_reset_us = 300;
		break;
	case 3: // SK6812
		led_t0h = 0.30; led_t0l = 0.90; led_t1h = 0.60; led_t1l = 0.60; led_reset_us = 80;
		break;
	case 4: // SK6815
		led_t0h = 0.30; led_t0l = 0.90; led_t1h = 0.60; led_t1l = 0.60; led_reset_us = 300;
		break;
	case 0:
	default: // Generic
		// Universal timing compromise: sits within the tolerance overlap zone of WS2812B, WS2815, SK6812, and SK6815.
		led_t0h = 0.35; led_t0l = 1.00; led_t1h = 0.95; led_t1l = 0.30; led_reset_us = 300;
		break;
	}
}

static led_strip_t *strip_find(int pin) {
	if (pin < 0) {
		return NULL; // -1 is the "free entry" sentinel, never a real pin
	}
	for (int i = 0; i < LED_STRIP_MAX; i++) {
		if (led_strip[i].pin == pin) {
			return &led_strip[i];
		}
	}
	return NULL;
}

// Build the shared channel, pointed at `pin`. Only the first call does work.
static bool led_chan_ensure(int pin, const char **err_reason) {
	if (led_chan != NULL) {
		return true;
	}

	led_pin_idle(pin);

	rmt_tx_channel_config_t tx_chan_config = {
			.clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
			.gpio_num = pin,
#if SOC_RMT_SUPPORT_DMA
			.mem_block_symbols = 256,
			.flags.with_dma = 1,
#else
			.mem_block_symbols = 64,
#endif
			.resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
			.trans_queue_depth = 4, // set the number of transactions that can be pending in the background
	};

	if (rmt_new_tx_channel(&tx_chan_config, &led_chan) != ESP_OK) {
		led_chan = NULL;
		if (err_reason) {
			*err_reason = "RMT channel init failed";
		}
		return false;
	}
	rmt_enable(led_chan);
	led_routed_pin = pin;
	led_find_tx_sig(pin);
	return true;
}

// Timing is baked into the encoder, so each preset in use needs its own.
static rmt_encoder_handle_t led_encoder_get(unsigned int timing,
		const char **err_reason) {
	if (timing >= LED_TIMING_MAX) {
		timing = 0;
	}
	if (led_enc[timing] == NULL) {
		led_timing_apply(timing);
		if (rmt_new_led_strip_encoder(&led_enc[timing]) != ESP_OK) {
			led_enc[timing] = NULL;
			if (err_reason) {
				*err_reason = "RMT encoder init failed";
			}
		}
	}
	return led_enc[timing];
}

// Point the channel at `pin`. Caller must have waited for the previous frame.
static bool led_route(int pin) {
	if (led_routed_pin == pin) {
		return true;
	}

	if (led_tx_sig < 0) {
		return false;
	}

	// Move the signal in the GPIO matrix with the channel left running. The
	// driver's rmt_tx_switch_gpio would do this too, but only from the INIT
	// state, so it costs a disable/enable - a DMA re-arm and a polled wait -
	// on every strip.
	if (led_routed_pin >= 0) {
		led_pin_idle(led_routed_pin);
	}
	esp_rom_gpio_pad_select_gpio((uint32_t)pin);
	gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
	esp_rom_gpio_connect_out_signal((uint32_t)pin, (uint32_t)led_tx_sig, false, false);
	led_routed_pin = pin;
	return true;
}

static void led_hw_teardown(void) {
	if (led_chan != NULL) {
		rmt_tx_wait_all_done(led_chan, 100);
		rmt_disable(led_chan);
		rmt_del_channel(led_chan);
		led_chan = NULL;
	}
	for (int i = 0; i < LED_TIMING_MAX; i++) {
		if (led_enc[i] != NULL) {
			rmt_del_encoder(led_enc[i]);
			led_enc[i] = NULL;
		}
	}
	led_routed_pin = -1;
	led_tx_sig = -1;
}

bool rgbled_init(int pin, unsigned int timing_preset) {
	rgbled_state_init_once();

	// Native libs reach this directly, so it cannot rely on the lisp wrapper.
	if (!utils_gpio_is_valid(pin)) {
		led_init_err = string_pin_invalid;
		return false;
	}

	if (timing_preset >= LED_TIMING_MAX) {
		timing_preset = 0;
	}

	led_lock();

	bool ok = false;
	bool is_new = false;
	led_strip_t *s = strip_find(pin);
	if (s == NULL) {
		for (int i = 0; i < LED_STRIP_MAX; i++) {
			if (led_strip[i].pin < 0) {
				s = &led_strip[i];
				break;
			}
		}
		if (s != NULL) {
			s->pin = pin;
			is_new = true;
		}
	}

	if (s == NULL) {
		led_init_err = "Too many LED strips";
	} else {
		s->timing = timing_preset;

		led_pin_idle(pin);

		const char *reason = NULL;
		if (led_chan_ensure(pin, &reason)
				&& led_encoder_get(timing_preset, &reason) != NULL) {
			ok = true;
		} else {
			led_init_err = reason ? reason : "LED strip init failed";
			if (is_new) {
				s->pin = -1;
			}
		}
	}

	led_unlock();
	return ok;
}

// hold_low leaves the pin LOW, else it resets. The shared channel is torn down once the last strip is gone.
static void rgbled_release(int pin, bool hold_low) {
	rgbled_state_init_once();
	led_lock();

	led_strip_t *s = strip_find(pin);
	if (s != NULL) {
		s->pin = -1;
	}

	if (led_chan != NULL && led_routed_pin == pin) {
		// Let the frame on the wire finish before the pin stops being driven.
		rmt_tx_wait_all_done(led_chan, 100);
		led_routed_pin = -1; // force the next update to re-route
	}

	if (hold_low) {
		led_pin_idle(pin);
	} else {
		gpio_reset_pin(pin);
	}

	bool any = false;
	for (int i = 0; i < LED_STRIP_MAX; i++) {
		if (led_strip[i].pin >= 0) {
			any = true; // strips remain, keep the channel
			break;
		}
	}
	if (!any) {
		led_hw_teardown();
	}

	led_unlock();
}

void rgbled_deinit(int pin) {
	rgbled_release(pin, false);
}

static lbm_value ext_rgbled_deinit(lbm_value *args, lbm_uint argn) {
	rgbled_state_init_once();

	// (rgbled-deinit [mode] [pin]): mode 1 holds the pin(s) LOW, 0 resets.
	// Without a pin every strip is released, matching the single-strip driver.
	int mode = 0;
	if (argn >= 1 && lbm_is_number(args[0])) {
		mode = lbm_dec_as_i32(args[0]);
	}
	bool hold_low = (mode == 1);

	led_lock();
	if (argn >= 2 && lbm_is_number(args[1])) {
		int pin = lbm_dec_as_i32(args[1]);
		rgbled_release(pin, hold_low);
		if (led_lisp_pin == pin) {
			led_lisp_pin = -1;
		}
	} else {
		for (int i = 0; i < LED_STRIP_MAX; i++) {
			if (led_strip[i].pin >= 0) {
				rgbled_release(led_strip[i].pin, hold_low);
			}
		}
		led_lisp_pin = -1;
	}
	led_unlock();

	return ENC_SYM_TRUE;
}

static lbm_value ext_rgbled_init(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	if (argn < 1 || argn > 2) {
		lbm_set_error_reason((char*)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}

	int pin = lbm_dec_as_i32(args[0]);
	if (!utils_gpio_is_valid(pin)) {
		lbm_set_error_reason(string_pin_invalid);
		return ENC_SYM_TERROR;
	}

	unsigned int timing_preset = 0;
	if (argn >= 2) {
		timing_preset = lbm_dec_as_u32(args[1]);
	}

	if (!rgbled_init(pin, timing_preset)) {
		lbm_set_error_reason(led_init_err);
		return ENC_SYM_EERROR;
	}

	led_lisp_pin = pin;

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
	if ((argn != 3 && argn != 4) || !lbm_is_array_rw(args[0]) ||
			!lbm_is_number(args[1]) || (!lbm_is_number(args[2]) && !lbm_is_list(args[2]))) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);
	if (array == NULL) {
		lbm_set_error_reason("LED buffer is NULL");
		return ENC_SYM_EERROR;
	}
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

void rgbled_update(int pin, uint8_t *data, size_t size) {
	rgbled_state_init_once();
	if (size < 1) {
		return;
	}

	// Held across the transmit: a concurrent deinit must not delete or re-point the shared channel between the route and the handover.
	led_lock();

	led_strip_t *s = strip_find(pin);
	rmt_encoder_handle_t enc = NULL;

	if (s != NULL && led_chan_ensure(pin, NULL)) {
		enc = led_encoder_get(s->timing, NULL);
	}

	if (enc != NULL) {
		// An in-flight frame belongs to whichever pin is still wired up.
		rmt_tx_wait_all_done(led_chan, 100);

		if (led_route(pin)) {
			rmt_transmit(led_chan, enc, data, size, &tx_config);
		}
	}

	led_unlock();
}

static lbm_value ext_rgbled_update(lbm_value *args, lbm_uint argn) {
	if (argn < 1 || argn > 2 || !lbm_is_array_r(args[0])) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	// (rgbled-update array)      -> the strip from the last rgbled-init
	// (rgbled-update array pin)  -> a specific strip
	int pin = led_lisp_pin;
	if (argn >= 2) {
		if (!lbm_is_number(args[1])) {
			lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
			return ENC_SYM_TERROR;
		}
		pin = lbm_dec_as_i32(args[1]);
	}

	led_lock();
	bool known = pin >= 0 && strip_find(pin) != NULL;
	led_unlock();
	if (!known) {
		lbm_set_error_reason("Please run rgbled-init first");
		return ENC_SYM_EERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);
	if (array == NULL) {
		lbm_set_error_reason("LED buffer is NULL");
		return ENC_SYM_EERROR;
	}

	rgbled_update(pin, (uint8_t*)array->data + 1, array->size - 1);

	return ENC_SYM_TRUE;
}

void lispif_load_rgbled_extensions(void) {
	if (led_mtx == NULL) {
		led_mtx = xSemaphoreCreateRecursiveMutexStatic(&led_mtx_buf);
	}

	lbm_add_extension("rgbled-init", ext_rgbled_init);
	lbm_add_extension("rgbled-deinit", ext_rgbled_deinit);
	lbm_add_extension("rgbled-buffer", ext_rgbled_color_buffer);
	lbm_add_extension("rgbled-color", ext_rgbled_color);
	lbm_add_extension("rgbled-update", ext_rgbled_update);
}
