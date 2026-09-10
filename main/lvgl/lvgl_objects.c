/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "lvgl_objects.h"

#include <stdbool.h>
#include <string.h>

#include "esp_heap_caps.h"

#define LVGL_HANDLE_SLOT_COUNT 128U
#define LVGL_HANDLE_SLOT_BITS  8U
#define LVGL_HANDLE_SLOT_MASK  ((1U << LVGL_HANDLE_SLOT_BITS) - 1U)

typedef struct {
	lvgl_handle_kind_t kind;
	uint16_t generation;
	union {
		lv_obj_t *object;
		lv_font_t *font;
		lv_image_dsc_t *image;
	} value;
	void *owned_data;
} lvgl_handle_slot_t;

static lvgl_handle_slot_t m_slots[LVGL_HANDLE_SLOT_COUNT];

static int32_t make_handle(uint32_t index) {
	return (int32_t)(((uint32_t)m_slots[index].generation <<
		LVGL_HANDLE_SLOT_BITS) | (index + 1U));
}

static lvgl_handle_slot_t *find_slot(int32_t handle,
		lvgl_handle_kind_t kind) {
	if (handle <= 0) {
		return NULL;
	}
	uint32_t raw = (uint32_t)handle;
	uint32_t encoded_index = raw & LVGL_HANDLE_SLOT_MASK;
	if (encoded_index == 0U) {
		return NULL;
	}
	uint32_t index = encoded_index - 1U;
	uint32_t generation = raw >> LVGL_HANDLE_SLOT_BITS;
	if (index >= LVGL_HANDLE_SLOT_COUNT ||
		m_slots[index].kind != kind ||
		m_slots[index].generation != generation) {
		return NULL;
	}
	return &m_slots[index];
}

static int32_t reserve_slot(lvgl_handle_kind_t kind) {
	for (uint32_t i = 0; i < LVGL_HANDLE_SLOT_COUNT; i++) {
		if (m_slots[i].kind == LVGL_HANDLE_KIND_NONE) {
			m_slots[i].generation++;
			if (m_slots[i].generation == 0U) {
				m_slots[i].generation = 1U;
			}
			m_slots[i].kind = kind;
			return make_handle(i);
		}
	}
	return 0;
}

void lvgl_objects_reset(void) {
	lv_obj_t *root_objects[LVGL_HANDLE_SLOT_COUNT];
	uint32_t root_count = 0U;
	bool active_is_owned = false;
	lv_obj_t *active = lv_screen_active();
	for (uint32_t i = 0; i < LVGL_HANDLE_SLOT_COUNT; i++) {
		if (m_slots[i].kind == LVGL_HANDLE_KIND_OBJECT &&
			m_slots[i].value.object) {
			if (m_slots[i].value.object == active) {
				active_is_owned = true;
			}
			/* Collect roots before deleting any of them. Once a root is
			 * deleted, all of its registered child pointers become stale. */
			if (lv_obj_get_parent(m_slots[i].value.object) == NULL) {
				root_objects[root_count++] = m_slots[i].value.object;
			}
		}
	}

	if (active_is_owned) {
		lv_obj_t *blank = lv_obj_create(NULL);
		if (blank) {
			lv_obj_set_style_bg_color(blank, lv_color_hex(0x000000),
				LV_PART_MAIN);
			lv_obj_set_style_bg_opa(blank, LV_OPA_COVER, LV_PART_MAIN);
			lv_screen_load(blank);
		}
	}

	/* Delete only registered root screens. Their children follow automatically. */
	for (uint32_t i = 0; i < root_count; i++) {
		lv_obj_delete(root_objects[i]);
	}

	for (uint32_t i = 0; i < LVGL_HANDLE_SLOT_COUNT; i++) {
		if (m_slots[i].kind == LVGL_HANDLE_KIND_FONT &&
			m_slots[i].value.font) {
			lv_binfont_destroy(m_slots[i].value.font);
		}
		if (m_slots[i].kind == LVGL_HANDLE_KIND_IMAGE &&
			m_slots[i].value.image) {
			heap_caps_free(m_slots[i].value.image);
		}
		if (m_slots[i].owned_data) {
			heap_caps_free(m_slots[i].owned_data);
		}
		m_slots[i].kind = LVGL_HANDLE_KIND_NONE;
		m_slots[i].value.object = NULL;
		m_slots[i].owned_data = NULL;
	}
}

int32_t lvgl_objects_add_object(lv_obj_t *object) {
	if (!object) {
		return 0;
	}
	int32_t handle = reserve_slot(LVGL_HANDLE_KIND_OBJECT);
	if (handle) {
		find_slot(handle, LVGL_HANDLE_KIND_OBJECT)->value.object = object;
	}
	return handle;
}

lv_obj_t *lvgl_objects_get_object(int32_t handle) {
	lvgl_handle_slot_t *slot = find_slot(handle, LVGL_HANDLE_KIND_OBJECT);
	return slot ? slot->value.object : NULL;
}

int32_t lvgl_objects_add_font(lv_font_t *font, void *owned_data) {
	if (!font || !owned_data) {
		return 0;
	}
	int32_t handle = reserve_slot(LVGL_HANDLE_KIND_FONT);
	if (handle) {
		lvgl_handle_slot_t *slot = find_slot(handle, LVGL_HANDLE_KIND_FONT);
		slot->value.font = font;
		slot->owned_data = owned_data;
	}
	return handle;
}

lv_font_t *lvgl_objects_get_font(int32_t handle) {
	lvgl_handle_slot_t *slot = find_slot(handle, LVGL_HANDLE_KIND_FONT);
	return slot ? slot->value.font : NULL;
}

int32_t lvgl_objects_add_image(lv_image_dsc_t *image, void *owned_data) {
	if (!image || !owned_data) {
		return 0;
	}
	int32_t handle = reserve_slot(LVGL_HANDLE_KIND_IMAGE);
	if (handle) {
		lvgl_handle_slot_t *slot = find_slot(handle, LVGL_HANDLE_KIND_IMAGE);
		slot->value.image = image;
		slot->owned_data = owned_data;
	}
	return handle;
}

lv_image_dsc_t *lvgl_objects_get_image(int32_t handle) {
	lvgl_handle_slot_t *slot = find_slot(handle, LVGL_HANDLE_KIND_IMAGE);
	return slot ? slot->value.image : NULL;
}
