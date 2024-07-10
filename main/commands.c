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

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <dirent.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "commands.h"
#include "datatypes.h"
#include "conf_general.h"
#include "comm_can.h"
#include "terminal.h"
#include "mempools.h"
#include "utils.h"

#ifdef OVR_CONF_PARSER_H
#include OVR_CONF_PARSER_H
#else
#include "confparser.h"
#endif

#ifdef OVR_CONF_XML_H
#include OVR_CONF_XML_H
#else
#include "confxml.h"
#endif

#include "packet.h"
#include "buffer.h"
#include "main.h"
#include "crc.h"
#include "comm_wifi.h"
#include "log.h"
#include "nmea.h"
#include "lispif.h"
#include "flash_helper.h"
#include "bms.h"
#include "imu.h"

#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_vfs.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_sleep.h"
#include "soc/rtc.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_wifi.h"

// Settings
#define PRINT_BUFFER_SIZE	400

// For double precision literals
#define D(x) 						((double)x##L)

// Private variables
static SemaphoreHandle_t print_mutex;
static bool init_done = false;

static const esp_partition_t *update_partition = NULL;
static esp_ota_handle_t update_handle = 0;

// Function pointers
static send_func_t send_func = 0;
static send_func_t send_func_can_fwd = 0;
static send_func_t send_func_blocking = 0;

// Blocking thread
static SemaphoreHandle_t block_sem;
static uint8_t blocking_thread_cmd_buffer[PACKET_MAX_PL_LEN];
static volatile unsigned int blocking_thread_cmd_len = 0;
static volatile bool is_blocking = false;

#if LOGS_ENABLED
volatile send_func_t stored_send_func;
static volatile send_func_t overwritten_send_func;
static volatile send_func_t temp_send_func;

void commands_start_send_func_overwrite(
    void (*new_send_func)(unsigned char *data, unsigned int len)
) {
	temp_send_func = new_send_func;
	overwritten_send_func = send_func;
	send_func = new_send_func;
}

void commands_restore_send_func() {
	if (send_func == temp_send_func) {
		send_func = overwritten_send_func;
	}
}

void commands_store_send_func() {
	stored_send_func = send_func;
}

#endif /* LOGS_ENABLED */

// Private functions
static void send_func_dummy(unsigned char *data, unsigned int len) {
	(void)data; (void)len;
}

static void block_task(void *arg) {
	for (;;) {
		is_blocking = false;

		xSemaphoreTake(block_sem, portMAX_DELAY);

		uint8_t *data = blocking_thread_cmd_buffer;
		unsigned int len = blocking_thread_cmd_len;

		COMM_PACKET_ID packet_id;
		static uint8_t send_buffer[512];

		packet_id = data[0];
		data++;
		len--;

		switch (packet_id) {
		case COMM_PING_CAN: {
			int32_t ind = 0;
			send_buffer[ind++] = COMM_PING_CAN;

			for (uint8_t i = 0;i < 255;i++) {
				HW_TYPE hw_type;
				if (comm_can_ping(i, &hw_type)) {
					send_buffer[ind++] = i;
				}
			}

			if (send_func_blocking) {
				send_func_blocking(send_buffer, ind);
			}
		} break;

		case COMM_TERMINAL_CMD:
			data[len] = '\0';
			terminal_process_string((char*)data);
			break;

		default:
			break;
		}

	}

	vTaskDelete(NULL);
}

void commands_init(void) {
	print_mutex = xSemaphoreCreateMutex();
	block_sem = xSemaphoreCreateBinary();
	xTaskCreatePinnedToCore(block_task, "comm_block", 2500, NULL, 7, NULL, tskNO_AFFINITY);
	init_done = true;
}

void commands_process_packet(unsigned char *data, unsigned int len,
		send_func_t reply_func) {
	if (!len) {
		return;
	}

	COMM_PACKET_ID packet_id;

	packet_id = data[0];
	data++;
	len--;

	if (packet_id != COMM_LISP_RMSG) {
		send_func = reply_func;
	}

	if (!send_func_can_fwd) {
		send_func_can_fwd = reply_func;
	}

	// Avoid calling invalid function pointer if it is null.
	if (!reply_func && packet_id != COMM_LISP_REPL_CMD) {
		reply_func = send_func_dummy;
	}

	switch (packet_id) {
	case COMM_FW_VERSION: {
		int32_t ind = 0;
		uint8_t send_buffer[65];
		send_buffer[ind++] = COMM_FW_VERSION;
		send_buffer[ind++] = FW_VERSION_MAJOR;
		send_buffer[ind++] = FW_VERSION_MINOR;

		strcpy((char*)(send_buffer + ind), HW_NAME);
		ind += strlen(HW_NAME) + 1;

		size_t size_bits = esp_efuse_get_field_size(ESP_EFUSE_MAC_FACTORY);
	    esp_efuse_read_field_blob(ESP_EFUSE_MAC_FACTORY, send_buffer + ind, size_bits);
	    ind += 6;
		memset(send_buffer + ind, 0, 6);
		ind += 6;

		send_buffer[ind++] = 0;
		send_buffer[ind++] = FW_TEST_VERSION_NUMBER;

		send_buffer[ind++] = HW_TYPE_CUSTOM_MODULE;
		send_buffer[ind++] = 1; // One custom config

		send_buffer[ind++] = 0; // No phase filters
		send_buffer[ind++] = 0; // No HW QML

		if (flash_helper_code_size(CODE_IND_QML) > 0) {
			send_buffer[ind++] = flash_helper_code_flags(CODE_IND_QML);
		} else {
			send_buffer[ind++] = 0;
		}

		send_buffer[ind++] = 0; // No NRF flags

		strcpy((char*)(send_buffer + ind), FW_NAME);
		ind += strlen(FW_NAME) + 1;

		buffer_append_uint32(send_buffer, main_calc_hw_crc(), &ind);

		reply_func(send_buffer, ind);
	} break;

	// TODO: Run crc check on new app, also make sure to skip duplicate packets
	case COMM_JUMP_TO_BOOTLOADER:
		if (update_handle != 0) {
			if (esp_ota_end(update_handle) == ESP_OK) {
				if (esp_ota_set_boot_partition(update_partition) == ESP_OK) {
					comm_wifi_disconnect();
					vTaskDelay(50 / portTICK_PERIOD_MS);

					esp_bluedroid_disable();
					esp_bt_controller_disable();
					esp_wifi_stop();

					// Here we must use esp_restart even though that does not play nicely
					// with USB. That is because we skip image validation in the bootloader
					// after deep sleep to get a faster boot time, allowing less power draw
					// in applications that need to wake up from deep sleep occasionally.
					esp_restart();
//					esp_sleep_enable_timer_wakeup(1000000);
//					esp_deep_sleep_start();
				}
			}
		}
		break;

	case COMM_ERASE_NEW_APP: {
		int32_t ind = 0;

		if (update_handle != 0) {
			esp_ota_abort(update_handle);
			update_handle = 0;
		}

		update_partition = esp_ota_get_next_update_partition(NULL);
		bool ok = false;
		if (update_partition != NULL) {
			esp_err_t res = esp_ota_begin(update_partition, buffer_get_uint32(data, &ind) - 6, &update_handle);
			ok = res == ESP_OK;
		}

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_ERASE_NEW_APP;
		send_buffer[ind++] = ok;
		reply_func(send_buffer, ind);
	} break;

	case COMM_WRITE_NEW_APP_DATA: {
		int32_t ind = 0;
		uint32_t new_app_offset = buffer_get_uint32(data, &ind);

		if (new_app_offset < 6) {
			ind += (6 - new_app_offset); // Skip size and crc
			new_app_offset = 0;
		} else {
			new_app_offset -= 6;
		}

		bool ok = false;
		if (update_handle != 0) {
			esp_err_t res = esp_ota_write_with_offset(update_handle, data + ind, len - ind, new_app_offset);
			ok = res == ESP_OK;
		}

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_WRITE_NEW_APP_DATA;
		send_buffer[ind++] = ok;
		buffer_append_uint32(send_buffer, new_app_offset, &ind);
		reply_func(send_buffer, ind);
	} break;

	case COMM_REBOOT: {
		comm_wifi_disconnect();

		esp_bluedroid_disable();
		esp_bt_controller_disable();
		esp_wifi_stop();

		// Deep sleep to reboot as that disconnects USB properly
//		esp_restart();
		esp_sleep_enable_timer_wakeup(1000000);
		esp_deep_sleep_start();
	} break;

	case COMM_FORWARD_CAN:
		send_func_can_fwd = reply_func;
		comm_can_send_buffer(data[0], data + 1, len - 1, 0);
		break;

	case COMM_CAN_FWD_FRAME: {
		int32_t ind = 0;
		uint32_t id = buffer_get_uint32(data, &ind);
		bool is_ext = data[ind++];

		if (is_ext) {
			comm_can_transmit_eid(id, data + ind, len - ind);
		} else {
			comm_can_transmit_sid(id, data + ind, len - ind);
		}
	} break;

	case COMM_GET_CUSTOM_CONFIG:
	case COMM_GET_CUSTOM_CONFIG_DEFAULT: {
		main_config_t *conf = calloc(1, sizeof(main_config_t));

		int conf_ind = data[0];

		if (conf_ind != 0) {
			break;
		}

		if (packet_id == COMM_GET_CUSTOM_CONFIG) {
			*conf = backup.config;
		} else {
#ifdef OVR_CONF_SET_DEFAULTS
			OVR_CONF_SET_DEFAULTS(conf);
#else
			confparser_set_defaults_main_config_t(conf);
#endif
		}

		uint8_t *send_buffer_global = mempools_get_packet_buffer();
		int32_t ind = 0;
		send_buffer_global[ind++] = packet_id;
		send_buffer_global[ind++] = conf_ind;
#ifdef OVR_CONF_SERIALIZE
		int32_t len = OVR_CONF_SERIALIZE(send_buffer_global + ind, conf);
#else
		int32_t len = confparser_serialize_main_config_t(send_buffer_global + ind, conf);
#endif
		commands_send_packet(send_buffer_global, len + ind);
		mempools_free_packet_buffer(send_buffer_global);

		free(conf);
	} break;

	case COMM_SET_CUSTOM_CONFIG: {
		main_config_t *conf = calloc(1, sizeof(main_config_t));
		*conf = backup.config;

		int conf_ind = data[0];

#ifdef OVR_CONF_DESERIALIZE
		if (conf_ind == 0 && OVR_CONF_DESERIALIZE(data + 1, conf)) {
#else
		if (conf_ind == 0 && confparser_deserialize_main_config_t(data + 1, conf)) {
#endif
			bool baud_changed = backup.config.can_baud_rate != conf->can_baud_rate;
			backup.config = *conf;

			if (baud_changed) {
				comm_can_update_baudrate();
			}

			main_store_backup_data();

			int32_t ind = 0;
			uint8_t send_buffer[50];
			send_buffer[ind++] = packet_id;
			reply_func(send_buffer, ind);
		} else {
			commands_printf("Warning: Could not set configuration");
		}

		free(conf);
	} break;

	case COMM_GET_CUSTOM_CONFIG_XML: {
		int32_t ind = 0;

		int conf_ind = data[ind++];

		if (conf_ind != 0) {
			break;
		}

		int32_t len_conf = buffer_get_int32(data, &ind);
		int32_t ofs_conf = buffer_get_int32(data, &ind);

		if ((len_conf + ofs_conf) > DATA_MAIN_CONFIG_T__SIZE || len_conf > (PACKET_MAX_PL_LEN - 10)) {
			break;
		}

		uint8_t *send_buffer_global = mempools_get_packet_buffer();
		ind = 0;
		send_buffer_global[ind++] = packet_id;
		send_buffer_global[ind++] = conf_ind;
		buffer_append_int32(send_buffer_global, DATA_MAIN_CONFIG_T__SIZE, &ind);
		buffer_append_int32(send_buffer_global, ofs_conf, &ind);
		memcpy(send_buffer_global + ind, data_main_config_t_ + ofs_conf, len_conf);
		ind += len_conf;
		reply_func(send_buffer_global, ind);
		mempools_free_packet_buffer(send_buffer_global);
	} break;

	case COMM_FILE_LIST: {
		int32_t ind = 0;
		char *path = (char*)data + ind;
		int path_len = strlen(path);
		ind += path_len + 1;
		char *from = (char*)data + ind;
		ind += strlen(from);

		uint8_t *send_buffer_global = mempools_get_packet_buffer();

		ind = 0;
		send_buffer_global[ind++] = packet_id;
		send_buffer_global[ind++] = 0; // Has more

		DIR *d;
		struct dirent *dir;
		bool from_found = strlen(from) == 0;

		char path_full[path_len + strlen("/sdcard/") + 1];
		strcpy(path_full, "/sdcard/");
		strcat(path_full, path);

		d = opendir(path_full);
		if (d) {
			while ((dir = readdir(d)) != NULL) {
				if (!from_found) {
					if (strcmp(dir->d_name, from) == 0) {
						from_found = true;
						continue;
					}
				} else {
					int len_f = strlen(dir->d_name);

					if ((ind + len_f) < 400) {
						send_buffer_global[ind++] = dir->d_type == DT_DIR;

						char path_file[strlen(path_full) + strlen(dir->d_name) + 2];
						strcpy(path_file, path_full);
						strcat(path_file, "/");
						strcat(path_file, dir->d_name);

						size_t size = 0;
						if (dir->d_type != DT_DIR) {
							FILE *f = fopen(path_file, "r");
							if (f) {
								fseek(f, 0, SEEK_END);
								size = ftell(f);
								fclose(f);
							}
						} else {
							DIR *d2 = opendir(path_file);
							if (d2) {
								struct dirent *dir2;
								while ((dir2 = readdir(d2)) != NULL) {
									size++;
								}
								closedir(d2);
							}
						}

						buffer_append_int32(send_buffer_global, size, &ind);

						strcpy((char*)send_buffer_global + ind, dir->d_name);
						ind += len_f + 1;
					} else {
						send_buffer_global[1] = 1; // There are more files to list
						break;
					}
				}
			}
			closedir(d);
		}

		reply_func(send_buffer_global, ind);
		mempools_free_packet_buffer(send_buffer_global);
	} break;

	case COMM_FILE_READ: {
		static FILE *f_last = 0;
		static int32_t f_last_offset = 0;
		static int32_t f_last_size = 0;
		uint8_t *wifi_buffer = 0;

		uint8_t *send_buffer = 0;
		size_t send_size = 400;

		void(*reply_func_raw)(unsigned char *data, unsigned int len) = 0;
		if (reply_func == comm_wifi_send_packet_local) {
			reply_func_raw = comm_wifi_send_raw_local;
		} else if (reply_func == comm_wifi_send_packet_hub) {
			reply_func_raw = comm_wifi_send_raw_hub;
		}

		if (reply_func_raw) {
			const int wifi_buffer_size = 4000;
			wifi_buffer = malloc(wifi_buffer_size);
			if (wifi_buffer) {
				send_buffer = wifi_buffer + 3;
				send_size = wifi_buffer_size - 100;
			} else {
				reply_func_raw = 0;
				send_buffer = mempools_get_packet_buffer();
			}
		} else {
			send_buffer = mempools_get_packet_buffer();
		}

		int32_t ind = 0;
		char *path = (char*)data + ind;
		int path_len = strlen(path);
		ind += path_len + 1;
		int32_t offset = buffer_get_int32(data, &ind);

		char path_full[path_len + strlen("/sdcard/") + 1];
		strcpy(path_full, "/sdcard/");
		strcat(path_full, path);

		ind = 0;
		send_buffer[ind++] = packet_id;
		buffer_append_int32(send_buffer, offset, &ind);

		if (!f_last || f_last_offset != offset) {
			if (f_last) {
				fclose(f_last);
			}

			f_last = fopen(path_full, "r");
			if (f_last) {
				fseek(f_last, 0, SEEK_END);
				f_last_size = ftell(f_last);
				fseek(f_last, offset, SEEK_SET);
				f_last_offset = offset;
			}
		}

		if (f_last) {
			buffer_append_int32(send_buffer, f_last_size, &ind);
			int32_t rd = read(fileno(f_last), send_buffer + ind, send_size);
			ind += rd;
			f_last_offset += rd;

			if (f_last_offset == f_last_size) {
				fclose(f_last);
				f_last = 0;
			}
		} else {
			buffer_append_int32(send_buffer, 0, &ind);
		}

		if (reply_func_raw) {
			unsigned short crc = crc16(send_buffer, ind);

			if (ind > 255) {
				wifi_buffer[0] = 3;
				wifi_buffer[1] = ind >> 8;
				wifi_buffer[2] = ind & 0xFF;
				ind += 3;
				wifi_buffer[ind++] = (uint8_t)(crc >> 8);
				wifi_buffer[ind++] = (uint8_t)(crc & 0xFF);
				wifi_buffer[ind++] = 3;
				reply_func_raw(wifi_buffer, ind);
			} else {
				wifi_buffer[1] = 2;
				wifi_buffer[2] = ind & 0xFF;
				ind += 3;
				wifi_buffer[ind++] = (uint8_t)(crc >> 8);
				wifi_buffer[ind++] = (uint8_t)(crc & 0xFF);
				wifi_buffer[ind++] = 3;
				reply_func_raw(wifi_buffer + 1, ind - 1);
			}

			free(wifi_buffer);
		} else {
			reply_func(send_buffer, ind);
			mempools_free_packet_buffer(send_buffer);
		}
	} break;

	case COMM_FILE_WRITE: {
		static FILE *f_last = 0;
		static int32_t f_last_offset = 0;

		int32_t ind = 0;
		char *path = (char*)data + ind;
		int path_len = strlen(path);
		ind += path_len + 1;
		int32_t offset = buffer_get_int32(data, &ind);
		int32_t size = buffer_get_int32(data, &ind);

		char path_full[path_len + strlen("/sdcard/") + 1];
		strcpy(path_full, "/sdcard/");
		strcat(path_full, path);

		bool ok = false;

		if (offset == 0) {
			if (f_last) {
				fclose(f_last);
			}

			f_last = fopen(path_full, "w");
			f_last_offset = 0;
		}

		if (f_last) {
			if (f_last_offset == offset) {
				ok = (len - ind) == fwrite(data + ind, 1, len - ind, f_last);
				if (ok) {
					f_last_offset += len - ind;
				} else {
					fclose(f_last);
					f_last = 0;
				}
			} else {
				// This was probably a retry if this is true, although that is not a safe assumption
				if (f_last_offset - (len - ind) == offset) {
					ok = true;
				}
			}
		}

		if (f_last && f_last_offset == size) {
			fclose(f_last);
			f_last = 0;
		}

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = packet_id;
		buffer_append_int32(send_buffer, offset, &ind);
		send_buffer[ind++] = ok;
		reply_func(send_buffer, ind);
	} break;

	case COMM_FILE_MKDIR: {
		int32_t ind = 0;
		char *path = (char*)data + ind;
		int path_len = strlen(path);
		ind += path_len + 1;

		char path_full[path_len + strlen("/sdcard/") + 1];
		strcpy(path_full, "/sdcard/");
		strcat(path_full, path);

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = packet_id;
		send_buffer[ind++] = mkdir(path_full, 0775) == 0;
		reply_func(send_buffer, ind);
	} break;

	case COMM_FILE_REMOVE: {
		int32_t ind = 0;
		char *path = (char*)data + ind;
		int path_len = strlen(path);
		ind += path_len + 1;

		char path_full[path_len + strlen("/sdcard/") + 1];
		strcpy(path_full, "/sdcard/");
		strcat(path_full, path);

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = packet_id;
		send_buffer[ind++] = utils_rmtree(path_full);
		reply_func(send_buffer, ind);
	} break;

	case COMM_LOG_START:
	case COMM_LOG_STOP:
	case COMM_LOG_CONFIG_FIELD:
	case COMM_LOG_DATA_F32:
	case COMM_LOG_DATA_F64: {
		log_process_packet(data - 1, len + 1);
	} break;

	case COMM_GET_GNSS: {
		int32_t ind = 0;
		uint32_t mask = buffer_get_uint16(data, &ind);

		nmea_state_t *g = nmea_get_state();

		ind = 0;
		uint8_t send_buffer[80];
		send_buffer[ind++] = packet_id;
		buffer_append_uint32(send_buffer, mask, &ind);

		if (mask & ((uint32_t)1 << 0)) { buffer_append_double64(send_buffer, g->gga.lat, D(1e16), &ind); }
		if (mask & ((uint32_t)1 << 1)) { buffer_append_double64(send_buffer, g->gga.lon, D(1e16), &ind); }
		if (mask & ((uint32_t)1 << 2)) { buffer_append_float32_auto(send_buffer, g->gga.height, &ind); }
		if (mask & ((uint32_t)1 << 3)) { buffer_append_float32_auto(send_buffer, g->rmc.speed, &ind); }
		if (mask & ((uint32_t)1 << 4)) { buffer_append_float32_auto(send_buffer, g->gga.h_dop, &ind); }
		if (mask & ((uint32_t)1 << 5)) { buffer_append_int32(send_buffer, g->gga.ms_today, &ind); }
		if (mask & ((uint32_t)1 << 6)) { buffer_append_int16(send_buffer, g->rmc.yy, &ind); }
		if (mask & ((uint32_t)1 << 7)) { send_buffer[ind++] = g->rmc.mo; }
		if (mask & ((uint32_t)1 << 8)) { send_buffer[ind++] = g->rmc.dd; }
		if (mask & ((uint32_t)1 << 9)) { buffer_append_float32_auto(send_buffer, -1, &ind); } // TODO: Store update time

		reply_func(send_buffer, ind);
	} break;

	case COMM_LISP_SET_RUNNING:
	case COMM_LISP_GET_STATS:
	case COMM_LISP_REPL_CMD:
	case COMM_LISP_STREAM_CODE:
	case COMM_LISP_RMSG: {
		lispif_process_cmd(data - 1, len + 1, reply_func);
		break;
	}

	case COMM_GET_QML_UI_APP:
	case COMM_LISP_READ_CODE: {
		int32_t ind = 0;

		int32_t len_qml = buffer_get_int32(data, &ind);
		int32_t ofs_qml = buffer_get_int32(data, &ind);

		int code_type = CODE_IND_QML;
		if (packet_id == COMM_LISP_READ_CODE) {
			code_type = CODE_IND_LISP;
		}

		int32_t qmlui_len = flash_helper_code_size(code_type);

		if (qmlui_len == 0) {
			ind = 0;
			uint8_t send_buffer[50];
			send_buffer[ind++] = packet_id;
			buffer_append_int32(send_buffer, 0, &ind);
			buffer_append_int32(send_buffer, 0, &ind);
			reply_func(send_buffer, ind);
			break;
		}

		if ((len_qml + ofs_qml) > qmlui_len || len_qml > (PACKET_MAX_PL_LEN - 10)) {
			break;
		}

		uint8_t *send_buffer_global = mempools_get_packet_buffer();
		ind = 0;
		send_buffer_global[ind++] = packet_id;
		buffer_append_int32(send_buffer_global, qmlui_len, &ind);
		buffer_append_int32(send_buffer_global, ofs_qml, &ind);
		flash_helper_code_data(code_type, ofs_qml, send_buffer_global + ind, len_qml);
		ind += len_qml;
		reply_func(send_buffer_global, ind);
		mempools_free_packet_buffer(send_buffer_global);
	} break;

	case COMM_QMLUI_ERASE:
	case COMM_LISP_ERASE_CODE: {
		int32_t ind = 0;
		int erase_size = -1;
		if (len >= 4) {
			erase_size = buffer_get_int32(data, &ind);
		}

		if (packet_id == COMM_LISP_ERASE_CODE) {
			// Only restart if erase size is not -2. This is a hack to maintain backwards compatibility.
			if (erase_size != -2) {
				lispif_restart(false, false, false);
			}
		}

		bool flash_res = flash_helper_erase_code(packet_id == COMM_QMLUI_ERASE ? CODE_IND_QML : CODE_IND_LISP, erase_size);

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = packet_id;
		send_buffer[ind++] = flash_res ? 1 : 0;
		reply_func(send_buffer, ind);
	} break;

	case COMM_QMLUI_WRITE:
	case COMM_LISP_WRITE_CODE: {
		int32_t ind = 0;
		uint32_t qmlui_offset = buffer_get_uint32(data, &ind);

		bool flash_res = flash_helper_write_code(packet_id == COMM_QMLUI_WRITE ? CODE_IND_QML : CODE_IND_LISP,
				qmlui_offset, data + ind, len - ind, 0);

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = packet_id;
		send_buffer[ind++] = flash_res ? 1 : 0;
		buffer_append_uint32(send_buffer, qmlui_offset, &ind);
		reply_func(send_buffer, ind);
	} break;

	case COMM_IO_BOARD_GET_ALL: {
		int32_t ind = 0;
		int id = buffer_get_int16(data, &ind);

		io_board_adc_values *adc_1_4 = comm_can_get_io_board_adc_1_4_id(id);
		io_board_adc_values *adc_5_8 = comm_can_get_io_board_adc_5_8_id(id);
		io_board_digial_inputs *digital_in = comm_can_get_io_board_digital_in_id(id);

		if (!adc_1_4 && !adc_5_8 && !digital_in) {
			break;
		}

		uint8_t send_buffer[70];
		ind = 0;
		send_buffer[ind++] = packet_id;
		buffer_append_int16(send_buffer, id, &ind);

		if (adc_1_4) {
			send_buffer[ind++] = 1;
			buffer_append_float32_auto(send_buffer, UTILS_AGE_S(adc_1_4->rx_time), &ind);
			buffer_append_float16(send_buffer, adc_1_4->adc_voltages[0], 1e2, &ind);
			buffer_append_float16(send_buffer, adc_1_4->adc_voltages[1], 1e2, &ind);
			buffer_append_float16(send_buffer, adc_1_4->adc_voltages[2], 1e2, &ind);
			buffer_append_float16(send_buffer, adc_1_4->adc_voltages[3], 1e2, &ind);
		}

		if (adc_5_8) {
			send_buffer[ind++] = 2;
			buffer_append_float32_auto(send_buffer, UTILS_AGE_S(adc_1_4->rx_time), &ind);
			buffer_append_float16(send_buffer, adc_5_8->adc_voltages[0], 1e2, &ind);
			buffer_append_float16(send_buffer, adc_5_8->adc_voltages[1], 1e2, &ind);
			buffer_append_float16(send_buffer, adc_5_8->adc_voltages[2], 1e2, &ind);
			buffer_append_float16(send_buffer, adc_5_8->adc_voltages[3], 1e2, &ind);
		}

		if (digital_in) {
			send_buffer[ind++] = 3;
			buffer_append_float32_auto(send_buffer, UTILS_AGE_S(adc_1_4->rx_time), &ind);
			buffer_append_uint32(send_buffer, (digital_in->inputs >> 32) & 0xFFFFFFFF, &ind);
			buffer_append_uint32(send_buffer, (digital_in->inputs >> 0) & 0xFFFFFFFF, &ind);
		}

		reply_func(send_buffer, ind);
	} break;

	case COMM_IO_BOARD_SET_PWM: {
		int32_t ind = 0;
		int id = buffer_get_int16(data, &ind);
		int channel = buffer_get_int16(data, &ind);
		float duty = buffer_get_float32_auto(data, &ind);
		comm_can_io_board_set_output_pwm(id, channel, duty);
	} break;

	case COMM_IO_BOARD_SET_DIGITAL: {
		int32_t ind = 0;
		int id = buffer_get_int16(data, &ind);
		int channel = buffer_get_int16(data, &ind);
		bool on = data[ind++];
		comm_can_io_board_set_output_digital(id, channel, on);
	} break;

	case COMM_CUSTOM_APP_DATA:
		lispif_process_custom_app_data(data, len);
		break;

	case COMM_BMS_GET_VALUES:
	case COMM_BMS_SET_CHARGE_ALLOWED:
	case COMM_BMS_SET_BALANCE_OVERRIDE:
	case COMM_BMS_RESET_COUNTERS:
	case COMM_BMS_FORCE_BALANCE:
	case COMM_BMS_ZERO_CURRENT_OFFSET: {
		bms_process_cmd(data - 1, len + 1, reply_func);
		break;
	}

	case COMM_GET_IMU_DATA: {
		int32_t ind = 0;
		uint8_t send_buffer[70];
		send_buffer[ind++] = packet_id;

		int32_t ind2 = 0;
		uint32_t mask = buffer_get_uint16(data, &ind2);

		float rpy[3], acc[3], gyro[3], mag[3], q[4];
		imu_get_rpy(rpy);
		imu_get_accel(acc);
		imu_get_gyro(gyro);
		imu_get_mag(mag);
		imu_get_quaternions(q);

		buffer_append_uint16(send_buffer, mask, &ind);

		if (mask & ((uint32_t)1 << 0)) {
			buffer_append_float32_auto(send_buffer, rpy[0], &ind);
		}
		if (mask & ((uint32_t)1 << 1)) {
			buffer_append_float32_auto(send_buffer, rpy[1], &ind);
		}
		if (mask & ((uint32_t)1 << 2)) {
			buffer_append_float32_auto(send_buffer, rpy[2], &ind);
		}

		if (mask & ((uint32_t)1 << 3)) {
			buffer_append_float32_auto(send_buffer, acc[0], &ind);
		}
		if (mask & ((uint32_t)1 << 4)) {
			buffer_append_float32_auto(send_buffer, acc[1], &ind);
		}
		if (mask & ((uint32_t)1 << 5)) {
			buffer_append_float32_auto(send_buffer, acc[2], &ind);
		}

		if (mask & ((uint32_t)1 << 6)) {
			buffer_append_float32_auto(send_buffer, gyro[0], &ind);
		}
		if (mask & ((uint32_t)1 << 7)) {
			buffer_append_float32_auto(send_buffer, gyro[1], &ind);
		}
		if (mask & ((uint32_t)1 << 8)) {
			buffer_append_float32_auto(send_buffer, gyro[2], &ind);
		}

		if (mask & ((uint32_t)1 << 9)) {
			buffer_append_float32_auto(send_buffer, mag[0], &ind);
		}
		if (mask & ((uint32_t)1 << 10)) {
			buffer_append_float32_auto(send_buffer, mag[1], &ind);
		}
		if (mask & ((uint32_t)1 << 11)) {
			buffer_append_float32_auto(send_buffer, mag[2], &ind);
		}

		if (mask & ((uint32_t)1 << 12)) {
			buffer_append_float32_auto(send_buffer, q[0], &ind);
		}
		if (mask & ((uint32_t)1 << 13)) {
			buffer_append_float32_auto(send_buffer, q[1], &ind);
		}
		if (mask & ((uint32_t)1 << 14)) {
			buffer_append_float32_auto(send_buffer, q[2], &ind);
		}
		if (mask & ((uint32_t)1 << 15)) {
			buffer_append_float32_auto(send_buffer, q[3], &ind);
		}

		send_buffer[ind++] = backup.config.controller_id;

		reply_func(send_buffer, ind);
	} break;

	// Blocking commands
	case COMM_TERMINAL_CMD:
	case COMM_PING_CAN:
		if (!is_blocking) {
			memcpy(blocking_thread_cmd_buffer, data - 1, len + 1);
			blocking_thread_cmd_len = len + 1;
			is_blocking = true;
			send_func_blocking = reply_func;
			xSemaphoreGive(block_sem);
		}
		break;

	default:
		break;
	}
}

/**
 * Send a packet using the last can fwd function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void commands_send_packet_can_last(unsigned char *data, unsigned int len) {
	if (send_func_can_fwd) {
		send_func_can_fwd(data, len);
	}
}

void commands_send_packet(unsigned char *data, unsigned int len) {
	if (send_func) {
		send_func(data, len);
	}
}

send_func_t commands_get_send_func(void) {
	return send_func;
}

void commands_set_send_func(send_func_t func) {
	send_func = func;
}

int commands_printf(const char* format, ...) {
	if (!init_done) {
		return 0;
	}

	xSemaphoreTake(print_mutex, portMAX_DELAY);

	va_list arg;
	va_start (arg, format);
	int len;

	char *print_buffer = malloc(PRINT_BUFFER_SIZE);

	print_buffer[0] = COMM_PRINT;
	len = vsnprintf(print_buffer + 1, (PRINT_BUFFER_SIZE - 1), format, arg);
	va_end (arg);

	int len_to_print = (len < (PRINT_BUFFER_SIZE - 1)) ? len + 1 : PRINT_BUFFER_SIZE;

	if(len > 0) {
		commands_send_packet((unsigned char*)print_buffer, len_to_print);
	}

	free(print_buffer);
	xSemaphoreGive(print_mutex);

	return len_to_print - 1;
}

int commands_printf_lisp(const char* format, ...) {
	if (!init_done) {
		return 0;
	}

	xSemaphoreTake(print_mutex, portMAX_DELAY);

	va_list arg;
	va_start (arg, format);
	int len;

	char *print_buffer = malloc(PRINT_BUFFER_SIZE);

	print_buffer[0] = COMM_LISP_PRINT;
	len = vsnprintf(print_buffer + 1, (PRINT_BUFFER_SIZE - 1), format, arg);
	va_end (arg);

	int len_to_print = (len < (PRINT_BUFFER_SIZE - 1)) ? len + 1 : PRINT_BUFFER_SIZE;

	if (len > 0) {
		if (print_buffer[len_to_print - 1] == '\n') {
			len_to_print--;
		}

		commands_send_packet((unsigned char*)print_buffer, len_to_print);
	}

	free(print_buffer);
	xSemaphoreGive(print_mutex);

	return len_to_print - 1;
}

void commands_init_plot(char *namex, char *namey) {
	int ind = 0;
	uint8_t *send_buffer_global = mempools_get_packet_buffer();
	send_buffer_global[ind++] = COMM_PLOT_INIT;
	memcpy(send_buffer_global + ind, namex, strlen(namex));
	ind += strlen(namex);
	send_buffer_global[ind++] = '\0';
	memcpy(send_buffer_global + ind, namey, strlen(namey));
	ind += strlen(namey);
	send_buffer_global[ind++] = '\0';
	commands_send_packet(send_buffer_global, ind);
	mempools_free_packet_buffer(send_buffer_global);
}

void commands_plot_add_graph(char *name) {
	int ind = 0;
	uint8_t *send_buffer_global = mempools_get_packet_buffer();
	send_buffer_global[ind++] = COMM_PLOT_ADD_GRAPH;
	memcpy(send_buffer_global + ind, name, strlen(name));
	ind += strlen(name);
	send_buffer_global[ind++] = '\0';
	commands_send_packet(send_buffer_global, ind);
	mempools_free_packet_buffer(send_buffer_global);
}

void commands_plot_set_graph(int graph) {
	int ind = 0;
	uint8_t buffer[2];
	buffer[ind++] = COMM_PLOT_SET_GRAPH;
	buffer[ind++] = graph;
	commands_send_packet(buffer, ind);
}

void commands_send_plot_points(float x, float y) {
	int32_t ind = 0;
	uint8_t buffer[10];
	buffer[ind++] = COMM_PLOT_DATA;
	buffer_append_float32_auto(buffer, x, &ind);
	buffer_append_float32_auto(buffer, y, &ind);
	commands_send_packet(buffer, ind);
}

void commands_send_app_data(unsigned char *data, unsigned int len) {
	int32_t index = 0;
	uint8_t *send_buffer_global = mempools_get_packet_buffer();
	send_buffer_global[index++] = COMM_CUSTOM_APP_DATA;
	memcpy(send_buffer_global + index, data, len);
	index += len;
	commands_send_packet(send_buffer_global, index);
	mempools_free_packet_buffer(send_buffer_global);
}
