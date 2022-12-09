/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

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

#include "hw_xp_t.h"

void hw_init(void) {
	    gpio_config_t io_conf = {};
	    io_conf.intr_type = GPIO_INTR_DISABLE;
	    io_conf.mode = GPIO_MODE_OUTPUT;
	    io_conf.pin_bit_mask = ((1ULL << LED_RED_PIN) | (1ULL << LED_BLUE_PIN));
	    io_conf.pull_down_en = 0;
	    io_conf.pull_up_en = 0;
	    gpio_config(&io_conf);

	    LED_RED_OFF();
	    LED_BLUE_OFF();
}
