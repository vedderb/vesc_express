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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "conf_general.h"
#include "comm_ble.h"
#include "comm_uart.h"
#include "comm_usb.h"
#include "comm_can.h"
#include "comm_wifi.h"
#include "commands.h"
#include "confparser.h"
#include "log.h"
#include "adc.h"
#include "ublox.h"
#include "nmea.h"
#include "terminal.h"
#include "main.h"
#include "mempools.h"
#include "lispif.h"

#include <string.h>
#include <sys/time.h>

// Global variables
volatile backup_data backup;

// Private functions
static void terminal_nmea(int argc, const char **argv);
static void terminal_ublox_reinit(int argc, const char **argv);

void app_main(void) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	settimeofday(&tv, NULL);

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		nvs_flash_erase();
		ret = nvs_flash_init();
	}

	{
		nvs_handle_t my_handle;
		nvs_open("vesc", NVS_READONLY, &my_handle);
		size_t required_size = 0;
		nvs_get_blob(my_handle, "backup", NULL, &required_size);

		memset((void*)&backup, 0, sizeof(backup));

		if (required_size == sizeof(backup_data)) {
			nvs_get_blob(my_handle, "backup", (void*)&backup, &required_size);
		}

		if (backup.controller_id_init_flag != VAR_INIT_CODE) {
			backup.controller_id = HW_DEFAULT_ID;
			backup.controller_id_init_flag = VAR_INIT_CODE;
		}

		if (backup.can_baud_rate_init_flag != VAR_INIT_CODE) {
			backup.can_baud_rate = CONF_CAN_BAUD_RATE;
			backup.can_baud_rate_init_flag = VAR_INIT_CODE;
		}

		if (backup.config_init_flag != MAIN_CONFIG_T_SIGNATURE) {
			confparser_set_defaults_main_config_t((main_config_t*)(&backup.config));
			backup.config_init_flag = MAIN_CONFIG_T_SIGNATURE;
			backup.config.controller_id = backup.controller_id;
			backup.config.can_baud_rate = backup.can_baud_rate;
		}

		nvs_close(my_handle);
	}

	mempools_init();
	commands_init();
	comm_usb_init();
	comm_can_init();

	if (backup.config.ble_mode != BLE_MODE_DISABLED) {
		comm_ble_init();
	}

	if (backup.config.wifi_mode != WIFI_MODE_DISABLED) {
		comm_wifi_init();
	}

	nmea_init();
	log_init();

	HW_INIT_HOOK();

#ifdef HW_HAS_ADC
	adc_init();
#endif

	lispif_init();

	//	comm_uart_init();
	ublox_init(false);

	terminal_register_command_callback(
			"nmea_info",
			"Print NMEA message information",
			0,
			terminal_nmea);

	terminal_register_command_callback(
			"ublox_reinit",
			"Re-initialize ublox gnss receiver",
			0,
			terminal_ublox_reinit);

	for (;;) {
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}

void main_store_backup_data(void) {
	nvs_handle_t my_handle;
	backup.controller_id = backup.config.controller_id;
	backup.can_baud_rate = backup.config.can_baud_rate;
	nvs_open("vesc", NVS_READWRITE, &my_handle);
	nvs_set_blob(my_handle, "backup", (void*)&backup, sizeof(backup_data));
	nvs_commit(my_handle);
	nvs_close(my_handle);
}

static void terminal_nmea(int argc, const char **argv) {
	(void)argc;(void)argv;
	nmea_state_t *s = nmea_get_state();

	commands_printf(
			"GGA Cnt   : %d\n"
			"GSV GP cnt: %d\n"
			"GSV GL cnt: %d\n"
			"RMC cnt   : %d\n"
			"Fix Type  : %s\n"
			"Num sats  : %d\n"
			"HDOP      : %.2f\n"
			"Lat       : %.8f\n"
			"Lon       : %.8f\n"
			"Height    : %f\n"
			"Time      : %02d-%02d-%02d %02d:%02d:%02d\n"
			"Last GGA  : %s"
			"Last GSV  : %s"
			"Last RMC  : %s\n",
			s->gga_cnt,
			s->gsv_gp_cnt,
			s->gsv_gl_cnt,
			s->rmc_cnt,
			nmea_fix_type(),
			s->gga.n_sat,
			s->gga.h_dop,
			s->gga.lat,
			s->gga.lon,
			s->gga.height,
			s->rmc.yy, s->rmc.mo, s->rmc.dd, s->rmc.hh, s->rmc.mm, s->rmc.ss,
			s->last_gga,
			s->last_gsv,
			s->last_rmc);
}

static void terminal_ublox_reinit(int argc, const char **argv) {
	(void)argc;(void)argv;
	commands_printf("Res: %d", ublox_init(true));
}
