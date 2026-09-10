/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef MAIN_DRIVERS_AXP2101_H_
#define MAIN_DRIVERS_AXP2101_H_

#include <stdbool.h>

#include "driver/i2c.h"
#include "esp_err.h"

esp_err_t axp2101_init(i2c_port_t port);
esp_err_t axp2101_enable_display_power(void);
bool axp2101_is_ready(void);

#endif /* MAIN_DRIVERS_AXP2101_H_ */
