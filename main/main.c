/*
	Copyright 2022 Benjamin Vedder      benjamin@vedder.se
	Copyright 2023 Rasmus Söderhielm    rasmus.soderhielm@gmail.com

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
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "esp_task_wdt.h"
#include "sdkconfig.h"

#include "conf_general.h"
#include "comm_ble.h"
#include "comm_uart.h"
#include "comm_usb.h"
#include "comm_can.h"
#include "comm_wifi.h"
#include "commands.h"
#include "flash_helper.h"
#include "crc.h"

#ifdef OVR_CONF_XML_H
#include OVR_CONF_XML_H
#else
#include "confxml.h"
#endif

#ifdef OVR_CONF_PARSER_H
#include OVR_CONF_PARSER_H
#else
#include "confparser.h"
#endif

#include "log.h"
#include "adc.h"
#include "ublox.h"
#include "nmea.h"
#include "terminal.h"
#include "main.h"
#include "mempools.h"
#include "lispif.h"
#include "bms.h"
#include "ble/custom_ble.h"

#include <string.h>
#include <sys/time.h>

#if defined(HW_APP_WDT_TIMEOUT_S) || defined(HW_APP_WDT_STARTUP_TIMEOUT_S)
#define HW_APP_WDT_AUTO_ENABLE 1
#else
#define HW_APP_WDT_AUTO_ENABLE 0
#endif

// Global variables
volatile backup_data backup;

// Private variables
volatile static bool init_done = false;
static bool app_wdt_initialized = false;
#ifdef HW_APP_WDT_STARTUP_TIMEOUT_S
static uint32_t app_wdt_timeout_s = HW_APP_WDT_STARTUP_TIMEOUT_S;
#elif defined(HW_APP_WDT_TIMEOUT_S)
static uint32_t app_wdt_timeout_s = HW_APP_WDT_TIMEOUT_S;
#else
static uint32_t app_wdt_timeout_s = 30;
#endif
static SemaphoreHandle_t app_wdt_mutex = NULL;

// Private functions
static void terminal_nmea(int argc, const char **argv);
static void terminal_ublox_reinit(int argc, const char **argv);
static esp_err_t main_task_wdt_init(void);
static esp_err_t main_task_wdt_reconfigure_locked(uint32_t timeout_s);

static uint32_t main_task_wdt_idle_core_mask(void) {
	uint32_t mask = 0;
#if defined(CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0) && CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
	mask |= (1U << 0);
#endif
#if defined(CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1) && CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1
	mask |= (1U << 1);
#endif
	return mask;
}

static esp_err_t main_task_wdt_init(void) {
#if CONFIG_ESP_TASK_WDT_EN
	if (!app_wdt_mutex) {
		app_wdt_mutex = xSemaphoreCreateMutex();
		if (!app_wdt_mutex) {
			return ESP_ERR_NO_MEM;
		}
	}

	if (app_wdt_initialized) {
		return ESP_OK;
	}

	xSemaphoreTake(app_wdt_mutex, portMAX_DELAY);
	esp_err_t res = main_task_wdt_reconfigure_locked(app_wdt_timeout_s);
	xSemaphoreGive(app_wdt_mutex);

	return res;
#else
	return ESP_ERR_NOT_SUPPORTED;
#endif
}

static esp_err_t main_task_wdt_reconfigure_locked(uint32_t timeout_s) {
#if CONFIG_ESP_TASK_WDT_EN
	if (timeout_s < 1) {
		return ESP_ERR_INVALID_ARG;
	}

	esp_task_wdt_config_t config = {
		.timeout_ms = timeout_s * 1000,
		.idle_core_mask = main_task_wdt_idle_core_mask(),
		.trigger_panic = true,
	};

	esp_err_t res;
	if (app_wdt_initialized) {
		res = esp_task_wdt_reconfigure(&config);
		if (res == ESP_ERR_INVALID_STATE) {
			app_wdt_initialized = false;
			res = esp_task_wdt_init(&config);
		}
	} else {
		res = esp_task_wdt_init(&config);
		if (res == ESP_ERR_INVALID_STATE) {
			res = esp_task_wdt_reconfigure(&config);
		}
	}

	if (res == ESP_OK) {
		app_wdt_initialized = true;
		app_wdt_timeout_s = timeout_s;
	}

	return res;
#else
	return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t main_task_wdt_configure(bool is_enabled, uint32_t timeout_s) {
	if (timeout_s > 0) {
		esp_err_t res = main_task_wdt_set_timeout(timeout_s);
		if (res != ESP_OK) {
			return res;
		}
	}

	return is_enabled ? main_task_wdt_enable() : main_task_wdt_disable();
}

esp_err_t main_task_wdt_enable(void) {
	return main_task_wdt_enable_task(NULL);
}

esp_err_t main_task_wdt_enable_task(TaskHandle_t task) {
#if CONFIG_ESP_TASK_WDT_EN
	esp_err_t res = main_task_wdt_init();
	if (res != ESP_OK) {
		return res;
	}

	xSemaphoreTake(app_wdt_mutex, portMAX_DELAY);

	res = esp_task_wdt_status(task);
	if (res != ESP_OK) {
		res = esp_task_wdt_add(task);
	}

	xSemaphoreGive(app_wdt_mutex);

	return res;
#else
	return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t main_task_wdt_disable(void) {
	return main_task_wdt_disable_task(NULL);
}

esp_err_t main_task_wdt_disable_task(TaskHandle_t task) {
#if CONFIG_ESP_TASK_WDT_EN
	if (!app_wdt_initialized || !app_wdt_mutex) {
		return ESP_OK;
	}

	xSemaphoreTake(app_wdt_mutex, portMAX_DELAY);

	esp_err_t res = esp_task_wdt_status(task);
	if (res == ESP_OK) {
		res = esp_task_wdt_delete(task);
	} else if (res == ESP_ERR_NOT_FOUND || res == ESP_ERR_INVALID_STATE) {
		res = ESP_OK;
	}

	xSemaphoreGive(app_wdt_mutex);

	return res;
#else
	return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t main_task_wdt_reset(void) {
#if CONFIG_ESP_TASK_WDT_EN
	return esp_task_wdt_reset();
#else
	return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t main_task_wdt_set_timeout(uint32_t timeout_s) {
	if (timeout_s < 1) {
		return ESP_ERR_INVALID_ARG;
	}

#if CONFIG_ESP_TASK_WDT_EN
	if (!app_wdt_mutex) {
		app_wdt_mutex = xSemaphoreCreateMutex();
		if (!app_wdt_mutex) {
			return ESP_ERR_NO_MEM;
		}
	}

	xSemaphoreTake(app_wdt_mutex, portMAX_DELAY);
	esp_err_t res = main_task_wdt_reconfigure_locked(timeout_s);
	xSemaphoreGive(app_wdt_mutex);

	return res;
#else
	return ESP_ERR_NOT_SUPPORTED;
#endif
}

bool main_task_wdt_is_enabled(void) {
#if CONFIG_ESP_TASK_WDT_EN
	return app_wdt_initialized && esp_task_wdt_status(NULL) == ESP_OK;
#else
	return false;
#endif
}

uint32_t main_task_wdt_get_timeout(void) {
	return app_wdt_timeout_s;
}

void app_main(void) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	settimeofday(&tv, NULL);

#if CONFIG_ESP_TASK_WDT_EN && HW_APP_WDT_AUTO_ENABLE
	(void)main_task_wdt_enable();
#endif

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
#ifdef OVR_CONF_SET_DEFAULTS
			OVR_CONF_SET_DEFAULTS((main_config_t*)(&backup.config));
#else
			confparser_set_defaults_main_config_t((main_config_t*)(&backup.config));
#endif
			backup.config_init_flag = MAIN_CONFIG_T_SIGNATURE;
			backup.config.controller_id = backup.controller_id;
			backup.config.can_baud_rate = backup.can_baud_rate;
		}

		nvs_close(my_handle);
	}

	(void)main_task_wdt_reset();

	adc_init();
	(void)main_task_wdt_reset();

#ifdef HW_EARLY_LBM_INIT
	HW_INIT_HOOK();
	lispif_init();
	HW_POST_LISPIF_HOOK();
	(void)main_task_wdt_reset();
#endif

	mempools_init();
	bms_init();
	commands_init();
#ifdef CAN_TX_GPIO_NUM
	comm_can_start(CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM);
#endif
	comm_usb_init();
	(void)main_task_wdt_reset();

	vTaskDelay(1);

#ifndef HW_IS_SLAVE
	switch (backup.config.ble_mode) {
		case BLE_MODE_DISABLED: {
			break;
		}
		case BLE_MODE_OPEN:
		case BLE_MODE_ENCRYPTED: {
			comm_ble_init();
			break;
		}
		case BLE_MODE_SCRIPTING: {
			custom_ble_init();
			break;
		}
	}

	if (backup.config.wifi_mode != WIFI_MODE_DISABLED) {
		comm_wifi_init();
	}
	(void)main_task_wdt_reset();
#endif

	nmea_init();
	log_init();
#if VESC_ENABLE_STORAGE
#ifdef SD_PIN_MOSI
	log_mount_card(SD_PIN_MOSI, SD_PIN_MISO, SD_PIN_SCK, SD_PIN_CS, SDMMC_FREQ_DEFAULT);
#endif
#ifdef NAND_PIN_MOSI
	log_mount_nand_flash(NAND_PIN_MOSI, NAND_PIN_MISO, NAND_PIN_SCK, NAND_PIN_CS, FLASH_FREQ_KHZ);
#endif
#endif
	(void)main_task_wdt_reset();

#ifndef HW_EARLY_LBM_INIT
	HW_INIT_HOOK();
	lispif_init();
	HW_POST_LISPIF_HOOK();
	(void)main_task_wdt_reset();
#endif

#ifndef HW_NO_UART
#ifdef HW_UART_COMM
	comm_uart_init(UART_TX, UART_RX, UART_NUM, UART_BAUDRATE);
#else
	ublox_init(false, 500, UART_NUM, UART_RX, UART_TX);
#endif
	(void)main_task_wdt_reset();
#endif

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

	init_done = true;

#if CONFIG_ESP_TASK_WDT_EN && HW_APP_WDT_AUTO_ENABLE && defined(HW_APP_WDT_TIMEOUT_S)
	(void)main_task_wdt_set_timeout(HW_APP_WDT_TIMEOUT_S);
	(void)main_task_wdt_reset();
#endif

#if CONFIG_ESP_TASK_WDT_EN && HW_APP_WDT_AUTO_ENABLE
	(void)main_task_wdt_disable();
#endif

	// Exit main to free up heap-space
	vTaskDelete(NULL);
}

uint32_t main_calc_hw_crc(void) {
	uint32_t crc = 0;

	crc = crc32_with_init(
			data_main_config_t_,
			DATA_MAIN_CONFIG_T__SIZE,
			crc);

	if (flash_helper_code_size(CODE_IND_QML) > 0) {
		crc = crc32_with_init(
				flash_helper_code_data_ptr(CODE_IND_QML),
				flash_helper_code_size(CODE_IND_QML),
				crc);
	}

	return crc;
}

void main_store_backup_data(void) {
	nvs_handle_t my_handle;
#ifndef HW_IS_SLAVE
	backup.controller_id = backup.config.controller_id;
	backup.can_baud_rate = backup.config.can_baud_rate;
#endif
	nvs_open("vesc", NVS_READWRITE, &my_handle);
	nvs_set_blob(my_handle, "backup", (void*)&backup, sizeof(backup_data));
	nvs_commit(my_handle);
	nvs_close(my_handle);
}

bool main_init_done(void) {
	return init_done;
}

void main_wait_until_init_done(void) {
	while (!init_done) {
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
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
			"Time      : %02d-%02d-%02d %02d:%02d:%02d\n",
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
			s->rmc.yy, s->rmc.mo, s->rmc.dd, s->rmc.hh, s->rmc.mm, s->rmc.ss
			);
}

static void terminal_ublox_reinit(int argc, const char **argv) {
	(void)argc;(void)argv;
	commands_printf("Res: %d", ublox_init(true, 500, UART_NUM, UART_RX, UART_TX));
}
