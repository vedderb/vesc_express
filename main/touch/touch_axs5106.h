#ifndef TOUCH_AXS5106_H_
#define TOUCH_AXS5106_H_

#include "hwi2c.h"
#include "lispif_touch_extensions.h"

esp_err_t touch_axs5106_init(i2c_port_num_t port, uint16_t width, uint16_t height, lispif_touch_driver_t *driver);
void touch_axs5106_set_transforms(bool swap_xy, bool mirror_x, bool mirror_y);

#endif
