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

#include "conf_general.h"
#include "terminal.h"
#include "commands.h"
#include "comm_can.h"
#include "comm_ble.h"
#include "ble/custom_ble.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>

#include "esp_bt.h"
#include "esp_wifi.h"
#include "comm_ble.h"
#include "comm_wifi.h"
#include "nvs_flash.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "utils.h"
#include "spi_flash_mmap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Settings
#define CALLBACK_LEN						40

// Private types
typedef struct _terminal_callback_struct {
	const char *command;
	const char *help;
	const char *arg_names;
	void(*cbf)(int argc, const char **argv);
} terminal_callback_struct;

typedef struct {
	UBaseType_t task_num;
	unsigned long task_run_time;
} taskinfo_struct;


// Private variables
static terminal_callback_struct callbacks[CALLBACK_LEN];
static int callback_write = 0;

static taskinfo_struct *prev_taskinfo = NULL;
static size_t prev_taskinfo_n = 0;
static uint32_t prev_time = 0;

const char* utils_hw_type_to_string(HW_TYPE hw) {
	switch (hw) {
	case HW_TYPE_VESC: return "HW_TYPE_VESC"; break;
	case HW_TYPE_VESC_BMS: return "HW_TYPE_VESC_BMS"; break;
	case HW_TYPE_CUSTOM_MODULE: return "HW_TYPE_CUSTOM_MODULE"; break;
	default: return "FAULT_HARDWARE"; break;
	}
}

void terminal_process_string(char *str) {
	enum { kMaxArgs = 64 };
	int argc = 0;
	char *argv[kMaxArgs];

	char *p2 = strtok(str, " ");
	while (p2 && argc < kMaxArgs) {
		argv[argc++] = p2;
		p2 = strtok(0, " ");
	}

	if (argc == 0) {
		commands_printf("No command received\n");
		return;
	}

	// force command argument to be lowercase
	for(int i = 0; argv[0][i] != '\0'; i++){
		argv[0][i] = tolower(argv[0][i]);
	}
	
	commands_printf("> %s", argv[0]);

	for (int i = 0;i < callback_write;i++) {
		if (callbacks[i].cbf != 0 && strcmp(argv[0], callbacks[i].command) == 0) {
			callbacks[i].cbf(argc, (const char**)argv);
			return;
		}
	}

	if (strcmp(argv[0], "threads") == 0) {
		int num_tasks = uxTaskGetNumberOfTasks();
		TaskStatus_t *tasks = malloc(num_tasks * sizeof(TaskStatus_t));
		uint32_t time_total;
		num_tasks = uxTaskGetSystemState(tasks, num_tasks, &time_total);

		const char *state_names[] = {"Running", "Ready", "Blocked", "Suspended", "Deleted", "Invalid"};

		commands_printf("task num    stack prio     state           name stackmin    cpu      ticks   dcpu     dticks");
		commands_printf("--------------------------------------------------------------------------------------------");

		taskinfo_struct *new_task_info = malloc(num_tasks * sizeof(taskinfo_struct));

		for (int i = 0; i < num_tasks;i++) {
			uint32_t total_run_time = tasks[i].ulRunTimeCounter;
			double total_run_time_percent = (100.0 * total_run_time) / (double)time_total;

			char delta_str[30];
			strcpy(delta_str, "     -          -");

			if (prev_taskinfo != NULL){
				for (int j = 0; j < prev_taskinfo_n; j++){
					if (tasks[i].xTaskNumber == prev_taskinfo[j].task_num){
						uint32_t run_time = tasks[i].ulRunTimeCounter - prev_taskinfo[j].task_run_time;
						double run_time_percent = (100.0 * run_time) / (double)(time_total - prev_time);
						snprintf(delta_str, 30, "%5.1f%% %10lu", run_time_percent, run_time);
						break;
					}
				}
			}

			commands_printf("%8d %.8lx %4lu %9s %14s %8d %5.1f%% %10lu %s",
					tasks[i].xTaskNumber,
					tasks[i].pxStackBase, tasks[i].uxBasePriority, state_names[tasks[i].eCurrentState],
					tasks[i].pcTaskName, tasks[i].usStackHighWaterMark,  total_run_time_percent, total_run_time, delta_str);
			// Update task info
			new_task_info[i].task_num = tasks[i].xTaskNumber;
			new_task_info[i].task_run_time = tasks[i].ulRunTimeCounter;

		}
		if (prev_taskinfo != NULL){
			free(prev_taskinfo);
			prev_taskinfo = NULL;
		}

		prev_taskinfo = new_task_info;
		prev_taskinfo_n = num_tasks;
		prev_time = time_total;

		free(tasks);
		commands_printf(" ");
	} else if (strcmp(argv[0], "mem") == 0) {
		nvs_stats_t s;
		nvs_get_stats(NULL, &s);

		commands_printf("NVS free          : %d", s.free_entries);
		commands_printf("NVS ns cnt        : %d", s.namespace_count);
		commands_printf("NVS tot           : %d", s.total_entries);
		commands_printf("NVS used          : %d", s.used_entries);

		commands_printf("Heap free         : %d", esp_get_free_heap_size());
		commands_printf("Heap free int.    : %d", esp_get_free_internal_heap_size());
		commands_printf("Heap min          : %d", esp_get_minimum_free_heap_size());
		commands_printf("mmap data free    : %d", spi_flash_mmap_get_free_pages(SPI_FLASH_MMAP_DATA));
		commands_printf("mmap inst free    : %d", spi_flash_mmap_get_free_pages(SPI_FLASH_MMAP_INST));

		commands_printf(" ");
	} else if (strcmp(argv[0], "can_devs") == 0) {
		commands_printf("CAN devices seen on the bus the past second:\n");
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg *msg = comm_can_get_status_msg_index(i);

			if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 1.0) {
				commands_printf("ID                   : %i", msg->id);
				commands_printf("RX Time              : %i", msg->rx_time);
				commands_printf("Age (milliseconds)   : %.2f", (double)(UTILS_AGE_S(msg->rx_time) * 1000.0));
				commands_printf("RPM                  : %.2f", (double)msg->rpm);
				commands_printf("Current              : %.2f", (double)msg->current);
				commands_printf("Duty                 : %.2f\n", (double)msg->duty);
			}
		}
	} else if (strcmp(argv[0], "hw_status") == 0) {
		commands_printf("Firmware          : %d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR);
		commands_printf("Hardware          : %s", HW_NAME);

		commands_printf("IDF Version       : %s", IDF_VER);

		commands_printf("BLE MTU           : %d", comm_ble_mtu_now());
		commands_printf("BLE Connected     : %d", comm_ble_is_connected());
		commands_printf("Custom BLE Started: %d", custom_ble_started());
		commands_printf("CAN RX Recoveries : %d", comm_can_get_rx_recovery_cnt());

		esp_ip4_addr_t ip = comm_wifi_get_ip();
		esp_ip4_addr_t ip_client = comm_wifi_get_ip_client();

		commands_printf("WIFI IP           : " IPSTR, IP2STR(&ip));
		commands_printf("WIFI Connected    : %d", comm_wifi_is_connected());
		commands_printf("WIFI Connecting   : %d", comm_wifi_is_connecting());
		commands_printf("WIFI Client IP    : " IPSTR, IP2STR(&ip_client));
		commands_printf("WIFI Client Con   : %d", comm_wifi_is_client_connected());

		uint8_t ch_primary;
		wifi_second_chan_t ch_second;
		esp_err_t ch_res = esp_wifi_get_channel(&ch_primary, &ch_second);

		if (ch_res == ESP_OK) {
			commands_printf("WIFI Channel      : %d", ch_primary);
		} else if (ch_res == ESP_ERR_WIFI_NOT_INIT) {
			commands_printf("WIFI Channel      : ESP_ERR_WIFI_NOT_INIT");
		} else {
			commands_printf("WIFI Channel      : error %d", ch_res);
		}

		const esp_partition_t *running = esp_ota_get_running_partition();
		if (running != NULL) {
			esp_app_desc_t running_app_info;
			if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
				commands_printf("App running ver   : %s", running_app_info.version);
				commands_printf("App running proj  : %s", running_app_info.project_name);
			}
		} else {
			commands_printf("Could not get running partition.");
		}

		commands_printf("Reset Reason      : %d", esp_reset_reason());

		commands_printf(" ");
	} else if (strcmp(argv[0], "can_scan") == 0) {
		bool found = false;
		for (int i = 0;i < 254;i++) {
			HW_TYPE hw_type;
			if (comm_can_ping(i, &hw_type)) {
				commands_printf("Found %s with ID: %d", utils_hw_type_to_string(hw_type), i);
				found = true;
			}
		}

		if (found) {
			commands_printf("Done\n");
		} else {
			commands_printf("No CAN devices found\n");
		}
	} else if (strcmp(argv[0], "uptime") == 0) {
		commands_printf("Uptime: %.2f s", (double)(utils_ms_tot() / 1000.0));
	} else if (strcmp(argv[0], "store_log_context") == 0) {
#if LOGS_ENABLED
		commands_store_send_func();
		commands_printf("stored send_func: %p", commands_get_send_func());
#else
		commands_printf("Debug logging is disabled for this firmware!");
#endif
	}

	// The help command
	else if (strcmp(argv[0], "help") == 0) {
		commands_printf("Valid commands are:");
		commands_printf("help");
		commands_printf("  Show this help.");

		commands_printf("threads");
		commands_printf("  List all threads.");

		commands_printf("mem");
		commands_printf("  Print memory usage.");

		commands_printf("can_devs");
		commands_printf("  Print all CAN devices seen on the bus the past second.");

		commands_printf("hw_status");
		commands_printf("  Print some hardware status information.");

		commands_printf("can_scan");
		commands_printf("  Scan CAN-bus using ping commands, and print all devices that are found.");

		commands_printf("uptime");
		commands_printf("  Prints how many seconds have passed since boot.");
		
		commands_printf("store_log_context");
		commands_printf("  Remember the current device and connection method (i.e. BLE, WiFi, USB, etc),\n"
		                "  and send future debug logs to it. Only usefull while developing the firmware."
#if !LOGS_ENABLED
                        "\n  (Disabled for this firmware)"
#endif	
		);

		for (int i = 0;i < callback_write;i++) {
			if (callbacks[i].cbf == 0) {
				continue;
			}

			if (callbacks[i].arg_names) {
				commands_printf("%s %s", callbacks[i].command, callbacks[i].arg_names);
			} else {
				commands_printf(callbacks[i].command);
			}

			if (callbacks[i].help) {
				commands_printf("  %s", callbacks[i].help);
			} else {
				commands_printf("  There is no help available for this command.");
			}
		}

		commands_printf(" ");
	} else {
		commands_printf("Invalid command: %s\n"
				"type help to list all available commands\n", argv[0]);
	}
}

/**
 * Register a custom command  callback to the terminal. If the command
 * is already registered the old command callback will be replaced.
 *
 * @param command
 * The command name.
 *
 * @param help
 * A help text for the command. Can be NULL.
 *
 * @param arg_names
 * The argument names for the command, e.g. [arg_a] [arg_b]
 * Can be NULL.
 *
 * @param cbf
 * The callback function for the command.
 */
void terminal_register_command_callback(
		const char* command,
		const char *help,
		const char *arg_names,
		void(*cbf)(int argc, const char **argv)) {

	int callback_num = callback_write;

	for (int i = 0;i < callback_write;i++) {
		// First check the address in case the same callback is registered more than once.
		if (callbacks[i].command == command) {
			callback_num = i;
			break;
		}

		// Check by string comparison.
		if (strcmp(callbacks[i].command, command) == 0) {
			callback_num = i;
			break;
		}

		// Check if the callback is empty (unregistered)
		if (callbacks[i].cbf == 0) {
			callback_num = i;
			break;
		}
	}

	callbacks[callback_num].command = command;
	callbacks[callback_num].help = help;
	callbacks[callback_num].arg_names = arg_names;
	callbacks[callback_num].cbf = cbf;

	if (callback_num == callback_write) {
		callback_write++;
		if (callback_write >= CALLBACK_LEN) {
			callback_write = 0;
		}
	}
}

void terminal_unregister_callback(void(*cbf)(int argc, const char **argv)) {
	for (int i = 0;i < callback_write;i++) {
		if (callbacks[i].cbf == cbf) {
			callbacks[i].cbf = 0;
		}
	}
}
