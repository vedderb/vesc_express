#ifndef MAIN_DISPLAY_DISP_SH8601_H_
#define MAIN_DISPLAY_DISP_SH8601_H_

#include <stdint.h>
#include <stdbool.h>
#include "lispif_disp_extensions.h"

void disp_sh8601_init(int pin_sd0, int pin_clk, int pin_cs, int pin_reset, int pin_dc, int clock_mhz);
void disp_sh8601_command(uint8_t command, const uint8_t *args, int argn);
bool disp_sh8601_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors);
void disp_sh8601_clear(uint32_t color);
void disp_sh8601_reset(void);

#endif /* MAIN_DISPLAY_DISP_SH8601_H_ */
