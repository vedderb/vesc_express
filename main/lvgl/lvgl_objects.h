/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef MAIN_LVGL_LVGL_OBJECTS_H_
#define MAIN_LVGL_LVGL_OBJECTS_H_

#include <stddef.h>
#include <stdint.h>

#include "lvgl.h"

typedef enum {
	LVGL_HANDLE_KIND_NONE = 0,
	LVGL_HANDLE_KIND_OBJECT,
	LVGL_HANDLE_KIND_FONT,
	LVGL_HANDLE_KIND_IMAGE,
} lvgl_handle_kind_t;

/*
 * Handles cross the LispBM boundary; native pointers never do. A generation
 * is encoded in every handle so stale package values cannot alias new LVGL
 * objects after a Lisp restart.
 */
void lvgl_objects_reset(void);

int32_t lvgl_objects_add_object(lv_obj_t *object);
lv_obj_t *lvgl_objects_get_object(int32_t handle);

int32_t lvgl_objects_add_font(lv_font_t *font, void *owned_data);
lv_font_t *lvgl_objects_get_font(int32_t handle);

int32_t lvgl_objects_add_image(lv_image_dsc_t *image, void *owned_data);
lv_image_dsc_t *lvgl_objects_get_image(int32_t handle);

#endif /* MAIN_LVGL_LVGL_OBJECTS_H_ */
