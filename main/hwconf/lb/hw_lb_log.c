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

#include "hw_lb_log.h"
#include "terminal.h"
#include "commands.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "rom/gpio.h"

// Variables
static int can_fault_cnt = 0;

static void hw_task(void *arg) {
	for (;;) {
		hw_clear_can_fault();
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

static void terminal_custom_info(int argc, const char **argv) {
	(void)argc; (void)argv;
	commands_printf("CAN Fault Cnt: %d", can_fault_cnt);
}

void hw_init(void) {
	    gpio_config_t io_conf = {};
	    io_conf.intr_type = GPIO_INTR_DISABLE;
	    io_conf.mode = GPIO_MODE_OUTPUT;
	    io_conf.pin_bit_mask = ((1ULL << CAN_EN_GPIO_NUM));
	    io_conf.pull_down_en = 0;
	    io_conf.pull_up_en = 0;
	    gpio_config(&io_conf);

	    xTaskCreatePinnedToCore(hw_task, "hw", 256, NULL, 6, NULL, tskNO_AFFINITY);

	    gpio_set_level(CAN_EN_GPIO_NUM, 0);

	    terminal_register_command_callback(
	    		"custom_info",
				"Print custom hw info.",
				0,
				terminal_custom_info);
}

void hw_clear_can_fault(void) {
	for (int i = 0;i < 50;i++) {
		vTaskDelay(1);
		if (gpio_get_level(CAN_RX_GPIO_NUM) != 0) {
			return;
		}
	}

	esp_rom_gpio_connect_out_signal(CAN_TX_GPIO_NUM, SIG_GPIO_OUT_IDX, false, false);

	for (int i = 0;i < 150;i++) {
		gpio_set_level(CAN_TX_GPIO_NUM, 1);
		vTaskDelay(1);
		gpio_set_level(CAN_TX_GPIO_NUM, 0);
		vTaskDelay(1);
	}

	can_fault_cnt++;

	esp_rom_gpio_connect_out_signal(CAN_TX_GPIO_NUM, TWAI_TX_IDX, false, false);
}
