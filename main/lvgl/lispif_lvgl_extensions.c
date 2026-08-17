/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "lispif_lvgl_extensions.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "display_backend.h"
#include "lispbm.h"
#include "lispif.h"
#include "lvgl.h"
#include "lvgl_display.h"
#include "lvgl_objects.h"
#include "lvgl_runtime.h"

#define LVGL_IMAGE_FILE_HEADER_SIZE 16U
#define LVGL_IMAGE_FILE_VERSION     1U
#define LVGL_IMAGE_FILE_FMT_RGB565A8 1U

static bool arguments_are_numbers(lbm_value *args, lbm_uint argn) {
	for (lbm_uint i = 0; i < argn; i++) {
		if (!lbm_is_number(args[i])) {
			return false;
		}
	}
	return true;
}

static void *copy_resource(const void *source, size_t size) {
	if (!source || size == 0U) {
		return NULL;
	}
	void *copy = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	if (!copy) {
		copy = heap_caps_malloc(size, MALLOC_CAP_8BIT);
	}
	if (copy) {
		memcpy(copy, source, size);
	}
	return copy;
}

static uint16_t read_u16_le(const uint8_t *data) {
	return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *data) {
	return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
		((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static lv_obj_t *decode_object(lbm_value value) {
	if (!lbm_is_number(value)) {
		return NULL;
	}
	return lvgl_objects_get_object(lbm_dec_as_i32(value));
}

static lv_font_t *decode_font(lbm_value value) {
	if (!lbm_is_number(value)) {
		return NULL;
	}
	return lvgl_objects_get_font(lbm_dec_as_i32(value));
}

static lv_image_dsc_t *decode_image(lbm_value value) {
	if (!lbm_is_number(value)) {
		return NULL;
	}
	return lvgl_objects_get_image(lbm_dec_as_i32(value));
}

static lbm_value encode_new_object(lv_obj_t *object) {
	if (!object) {
		return ENC_SYM_MERROR;
	}
	int32_t handle = lvgl_objects_add_object(object);
	if (!handle) {
		lv_obj_delete(object);
		return ENC_SYM_MERROR;
	}
	return lbm_enc_i32(handle);
}

static lbm_value ext_lv_ready(lbm_value *args, lbm_uint argn) {
	(void)args;
	return argn == 0U && lvgl_runtime_is_ready() && lvgl_display_is_ready() ?
		ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_lv_reset(lbm_value *args, lbm_uint argn) {
	(void)args;
	if (argn != 0U || !lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	lvgl_objects_reset();
	lvgl_runtime_unlock();
	return ENC_SYM_TRUE;
}

static lbm_value ext_lv_display_brightness(lbm_value *args, lbm_uint argn) {
	if (argn != 1U || !lbm_is_number(args[0])) {
		return ENC_SYM_TERROR;
	}
	int32_t brightness = lbm_dec_as_i32(args[0]);
	if (brightness < 0 || brightness > 100) {
		return ENC_SYM_EERROR;
	}
	return display_backend_set_brightness((uint8_t)brightness) == ESP_OK ?
		ENC_SYM_TRUE : ENC_SYM_EERROR;
}

static lbm_value ext_lv_screen_create(lbm_value *args, lbm_uint argn) {
	(void)args;
	if (argn != 0U || !lvgl_runtime_lock()) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *screen = lv_obj_create(NULL);
	if (screen) {
		lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
	}
	lbm_value result = encode_new_object(screen);
	lvgl_runtime_unlock();
	return result;
}

typedef lv_obj_t *(*object_create_fn)(lv_obj_t *parent);

static lbm_value create_child(lbm_value *args, lbm_uint argn,
		object_create_fn create_fn) {
	if (argn != 1U) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *parent = decode_object(args[0]);
	if (!parent || !lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	lbm_value result = encode_new_object(create_fn(parent));
	lvgl_runtime_unlock();
	return result;
}

static lbm_value ext_lv_object_create(lbm_value *args, lbm_uint argn) {
	return create_child(args, argn, lv_obj_create);
}

static lbm_value ext_lv_label_create(lbm_value *args, lbm_uint argn) {
	return create_child(args, argn, lv_label_create);
}

static lbm_value ext_lv_arc_create(lbm_value *args, lbm_uint argn) {
	return create_child(args, argn, lv_arc_create);
}

static lbm_value ext_lv_image_create(lbm_value *args, lbm_uint argn) {
	return create_child(args, argn, lv_image_create);
}

static lbm_value ext_lv_screen_load(lbm_value *args, lbm_uint argn) {
	if (argn != 1U) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *screen = decode_object(args[0]);
	if (!screen || !lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	if (lv_obj_get_parent(screen) != NULL) {
		lvgl_runtime_unlock();
		return ENC_SYM_EERROR;
	}
	/* The package replaces the temporary reset screen (or boot smoke screen),
	 * so let LVGL dispose of that previous active root after the switch. */
	lv_screen_load_anim(screen, LV_SCREEN_LOAD_ANIM_NONE, 0, 0, true);
	lvgl_runtime_unlock();
	return ENC_SYM_TRUE;
}

static lbm_value ext_lv_object_set_size(lbm_value *args, lbm_uint argn) {
	if (argn != 3U || !arguments_are_numbers(args, argn)) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *object = decode_object(args[0]);
	if (!object || !lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	/* Lisp packages use -1 as a compact, stable content-size sentinel. LVGL
	 * 9 encodes LV_SIZE_CONTENT as an internal tagged coordinate rather than
	 * the LVGL 8-era value, so translate it at the native API boundary. */
	int32_t width = lbm_dec_as_i32(args[1]);
	int32_t height = lbm_dec_as_i32(args[2]);
	lv_obj_set_size(object,
		width == -1 ? LV_SIZE_CONTENT : width,
		height == -1 ? LV_SIZE_CONTENT : height);
	lvgl_runtime_unlock();
	return ENC_SYM_TRUE;
}

static lbm_value ext_lv_object_align(lbm_value *args, lbm_uint argn) {
	if (argn != 4U || !arguments_are_numbers(args, argn)) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *object = decode_object(args[0]);
	int32_t align = lbm_dec_as_i32(args[1]);
	if (!object || align < LV_ALIGN_DEFAULT || align > LV_ALIGN_OUT_RIGHT_BOTTOM ||
		!lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	lv_obj_align(object, (lv_align_t)align, lbm_dec_as_i32(args[2]),
		lbm_dec_as_i32(args[3]));
	lvgl_runtime_unlock();
	return ENC_SYM_TRUE;
}

static lbm_value ext_lv_object_remove_style_all(lbm_value *args,
		lbm_uint argn) {
	if (argn != 1U) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *object = decode_object(args[0]);
	if (!object || !lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	lv_obj_remove_style_all(object);
	lvgl_runtime_unlock();
	return ENC_SYM_TRUE;
}

static lbm_value ext_lv_object_style_bg(lbm_value *args, lbm_uint argn) {
	if (argn != 4U || !arguments_are_numbers(args, argn)) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *object = decode_object(args[0]);
	if (!object || !lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	lv_obj_set_style_bg_color(object, lv_color_hex(lbm_dec_as_u32(args[1])),
		LV_PART_MAIN);
	lv_obj_set_style_bg_opa(object, lbm_dec_as_i32(args[2]), LV_PART_MAIN);
	lv_obj_set_style_radius(object, lbm_dec_as_i32(args[3]), LV_PART_MAIN);
	lvgl_runtime_unlock();
	return ENC_SYM_TRUE;
}

static lbm_value ext_lv_object_style_border(lbm_value *args, lbm_uint argn) {
	if (argn != 4U || !arguments_are_numbers(args, argn)) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *object = decode_object(args[0]);
	if (!object || !lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	lv_obj_set_style_border_color(object,
		lv_color_hex(lbm_dec_as_u32(args[1])), LV_PART_MAIN);
	lv_obj_set_style_border_opa(object, lbm_dec_as_i32(args[2]),
		LV_PART_MAIN);
	lv_obj_set_style_border_width(object, lbm_dec_as_i32(args[3]),
		LV_PART_MAIN);
	lvgl_runtime_unlock();
	return ENC_SYM_TRUE;
}

static lbm_value ext_lv_label_text(lbm_value *args, lbm_uint argn) {
	if (argn != 2U) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *label = decode_object(args[0]);
	char *text = lbm_dec_str(args[1]);
	if (!label || !text || !lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	lv_label_set_text(label, text);
	lvgl_runtime_unlock();
	return ENC_SYM_TRUE;
}

static lbm_value ext_lv_label_style(lbm_value *args, lbm_uint argn) {
	if (argn != 4U || !arguments_are_numbers(args, argn)) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *label = decode_object(args[0]);
	int32_t font_handle = lbm_dec_as_i32(args[2]);
	lv_font_t *font = font_handle == 0 ? NULL : decode_font(args[2]);
	int32_t text_align = lbm_dec_as_i32(args[3]);
	if (!label || (font_handle != 0 && !font) ||
		text_align < LV_TEXT_ALIGN_AUTO || text_align > LV_TEXT_ALIGN_RIGHT ||
		!lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	lv_obj_set_style_text_color(label, lv_color_hex(lbm_dec_as_u32(args[1])),
		LV_PART_MAIN);
	if (font) {
		lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
	}
	lv_obj_set_style_text_align(label, (lv_text_align_t)text_align,
		LV_PART_MAIN);
	lvgl_runtime_unlock();
	return ENC_SYM_TRUE;
}

static lbm_value ext_lv_font_load(lbm_value *args, lbm_uint argn) {
	if (argn != 1U || !lbm_is_array_r(args[0])) {
		return ENC_SYM_TERROR;
	}
	lbm_array_header_t *array = lbm_dec_array_r(args[0]);
	if (!array || array->size == 0U) {
		return ENC_SYM_TERROR;
	}
	void *copy = copy_resource(array->data, array->size);
	if (!copy) {
		return ENC_SYM_MERROR;
	}
	if (!lvgl_runtime_lock()) {
		heap_caps_free(copy);
		return ENC_SYM_EERROR;
	}
	lv_font_t *font = lv_binfont_create_from_buffer(copy, array->size);
	int32_t handle = lvgl_objects_add_font(font, copy);
	if (!handle) {
		if (font) {
			lv_binfont_destroy(font);
		}
		heap_caps_free(copy);
	}
	lvgl_runtime_unlock();
	return handle ? lbm_enc_i32(handle) : ENC_SYM_MERROR;
}

static lbm_value ext_lv_image_load(lbm_value *args, lbm_uint argn) {
	if (argn != 1U || !lbm_is_array_r(args[0])) {
		return ENC_SYM_TERROR;
	}
	lbm_array_header_t *array = lbm_dec_array_r(args[0]);
	if (!array || array->size < LVGL_IMAGE_FILE_HEADER_SIZE) {
		return ENC_SYM_TERROR;
	}
	const uint8_t *file = (const uint8_t *)array->data;
	if (memcmp(file, "LVIM", 4) != 0 || file[4] != LVGL_IMAGE_FILE_VERSION ||
		file[5] != LVGL_IMAGE_FILE_FMT_RGB565A8) {
		return ENC_SYM_TERROR;
	}
	uint16_t width = read_u16_le(file + 8);
	uint16_t height = read_u16_le(file + 10);
	uint32_t data_size = read_u32_le(file + 12);
	if (width == 0U || height == 0U ||
		data_size != (uint32_t)width * height * 3U ||
		data_size > array->size - LVGL_IMAGE_FILE_HEADER_SIZE) {
		return ENC_SYM_TERROR;
	}
	void *copy = copy_resource(file + LVGL_IMAGE_FILE_HEADER_SIZE, data_size);
	if (!copy) {
		return ENC_SYM_MERROR;
	}
	lv_image_dsc_t *image = heap_caps_calloc(1, sizeof(*image),
		MALLOC_CAP_8BIT);
	if (!image) {
		heap_caps_free(copy);
		return ENC_SYM_MERROR;
	}
	image->header.magic = LV_IMAGE_HEADER_MAGIC;
	image->header.cf = LV_COLOR_FORMAT_RGB565A8;
	image->header.w = width;
	image->header.h = height;
	image->data_size = data_size;
	image->data = copy;
	int32_t handle = lvgl_objects_add_image(image, copy);
	if (!handle) {
		heap_caps_free(image);
		heap_caps_free(copy);
	}
	return handle ? lbm_enc_i32(handle) : ENC_SYM_MERROR;
}

static lbm_value ext_lv_image_set_src(lbm_value *args, lbm_uint argn) {
	if (argn != 2U) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *object = decode_object(args[0]);
	lv_image_dsc_t *image = decode_image(args[1]);
	if (!object || !image || !lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	lv_image_set_src(object, image);
	lvgl_runtime_unlock();
	return ENC_SYM_TRUE;
}

static lbm_value ext_lv_image_recolor(lbm_value *args, lbm_uint argn) {
	if (argn != 3U || !arguments_are_numbers(args, argn)) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *object = decode_object(args[0]);
	if (!object || !lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	lv_obj_set_style_image_recolor(object,
		lv_color_hex(lbm_dec_as_u32(args[1])), LV_PART_MAIN);
	lv_obj_set_style_image_recolor_opa(object, lbm_dec_as_i32(args[2]),
		LV_PART_MAIN);
	lvgl_runtime_unlock();
	return ENC_SYM_TRUE;
}

static lbm_value ext_lv_arc_config(lbm_value *args, lbm_uint argn) {
	if (argn != 6U || !arguments_are_numbers(args, argn)) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *arc = decode_object(args[0]);
	int32_t mode = lbm_dec_as_i32(args[5]);
	if (!arc || mode < LV_ARC_MODE_NORMAL || mode > LV_ARC_MODE_REVERSE ||
		!lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	lv_arc_set_range(arc, lbm_dec_as_i32(args[1]), lbm_dec_as_i32(args[2]));
	lv_arc_set_bg_angles(arc, lbm_dec_as_i32(args[3]),
		lbm_dec_as_i32(args[4]));
	lv_arc_set_mode(arc, (lv_arc_mode_t)mode);
	lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
	lvgl_runtime_unlock();
	return ENC_SYM_TRUE;
}

static lbm_value ext_lv_arc_value(lbm_value *args, lbm_uint argn) {
	if (argn != 2U || !arguments_are_numbers(args, argn)) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *arc = decode_object(args[0]);
	if (!arc || !lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	lv_arc_set_value(arc, lbm_dec_as_i32(args[1]));
	lvgl_runtime_unlock();
	return ENC_SYM_TRUE;
}

static lbm_value ext_lv_arc_style(lbm_value *args, lbm_uint argn) {
	if (argn != 4U || !arguments_are_numbers(args, argn)) {
		return ENC_SYM_TERROR;
	}
	lv_obj_t *arc = decode_object(args[0]);
	if (!arc || !lvgl_runtime_lock()) {
		return ENC_SYM_EERROR;
	}
	lv_obj_set_style_arc_color(arc, lv_color_hex(lbm_dec_as_u32(args[1])),
		LV_PART_MAIN);
	lv_obj_set_style_arc_opa(arc, LV_OPA_COVER, LV_PART_MAIN);
	lv_obj_set_style_arc_width(arc, lbm_dec_as_i32(args[3]), LV_PART_MAIN);
	lv_obj_set_style_arc_rounded(arc, false, LV_PART_MAIN);
	lv_obj_set_style_arc_color(arc, lv_color_hex(lbm_dec_as_u32(args[2])),
		LV_PART_INDICATOR);
	lv_obj_set_style_arc_opa(arc, LV_OPA_COVER, LV_PART_INDICATOR);
	lv_obj_set_style_arc_width(arc, lbm_dec_as_i32(args[3]),
		LV_PART_INDICATOR);
	lv_obj_set_style_arc_rounded(arc, false, LV_PART_INDICATOR);
	lvgl_runtime_unlock();
	return ENC_SYM_TRUE;
}

static void load_extensions(bool main_found) {
	(void)main_found;
	if (!lvgl_runtime_is_ready() || !lvgl_display_is_ready()) {
		return;
	}
	if (lvgl_runtime_lock()) {
		lvgl_objects_reset();
		lvgl_runtime_unlock();
	}

	lbm_add_extension("lv-ready", ext_lv_ready);
	lbm_add_extension("lv-reset", ext_lv_reset);
	lbm_add_extension("lv-display-brightness", ext_lv_display_brightness);
	lbm_add_extension("lv-screen-create", ext_lv_screen_create);
	lbm_add_extension("lv-object-create", ext_lv_object_create);
	lbm_add_extension("lv-label-create", ext_lv_label_create);
	lbm_add_extension("lv-arc-create", ext_lv_arc_create);
	lbm_add_extension("lv-image-create", ext_lv_image_create);
	lbm_add_extension("lv-screen-load", ext_lv_screen_load);
	lbm_add_extension("lv-object-set-size", ext_lv_object_set_size);
	lbm_add_extension("lv-object-align", ext_lv_object_align);
	lbm_add_extension("lv-object-remove-style-all", ext_lv_object_remove_style_all);
	lbm_add_extension("lv-object-style-bg", ext_lv_object_style_bg);
	lbm_add_extension("lv-object-style-border", ext_lv_object_style_border);
	lbm_add_extension("lv-label-text", ext_lv_label_text);
	lbm_add_extension("lv-label-style", ext_lv_label_style);
	lbm_add_extension("lv-font-load", ext_lv_font_load);
	lbm_add_extension("lv-image-load", ext_lv_image_load);
	lbm_add_extension("lv-image-set-src", ext_lv_image_set_src);
	lbm_add_extension("lv-image-recolor", ext_lv_image_recolor);
	lbm_add_extension("lv-arc-config", ext_lv_arc_config);
	lbm_add_extension("lv-arc-value", ext_lv_arc_value);
	lbm_add_extension("lv-arc-style", ext_lv_arc_style);
}

void lispif_lvgl_extensions_init(void) {
	lispif_add_ext_load_callback(load_extensions);
}
