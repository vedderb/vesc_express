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

#include "log.h"
#include "conf_general.h"
#include "nmea.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_vfs.h"
#include "buffer.h"
#include "utils.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <dirent.h>
#include <unistd.h>

typedef struct {
	char key[25];
	char name[30];
	char unit[10];
	int8_t precision;
	bool is_relative;
	bool is_timestamp;
	double value;
	bool updated;
} log_header;

#define LOG_MAX_FIELDS		120

// Private variables
static sdmmc_host_t m_host = SDSPI_HOST_DEFAULT();
static sdmmc_card_t *m_card = 0;

static volatile log_header m_headers[LOG_MAX_FIELDS];

static log_header m_header_ts;
static log_header m_header_ts_gnss;
static log_header m_header_lat;
static log_header m_header_lon;
static log_header m_header_alt;
static log_header m_header_hacc;
static log_header m_header_hvel;

static volatile int m_field_num = 0;
static volatile float m_rate_hz = 10.0;
static volatile bool m_append_time = false;
static volatile bool m_append_gnss = false;
static volatile bool m_append_gnss_time = false;

static void print_header(log_header *h, FILE *file) {
	fprintf(file, "%s:%s:%s:%d:%d:%d",
			h->key, h->name, h->unit,
			h->precision, h->is_relative, h->is_timestamp);
}

static void log_task(void *arg) {
	FILE *f_log = 0;
	int gga_cnt_last = 0;
	int rmc_cnt_last = 0;
	int64_t ms_last = utils_ms_tot();
	TickType_t tick_last_fsync = xTaskGetTickCount();

	for (;;) {
		if (!m_card) {
			vTaskDelay(10);
			continue;
		}

		nmea_state_t *s = nmea_get_state();

		bool date_valid = true;
		if (s->rmc.yy < 0 || s->rmc.mo < 0 || s->rmc.dd < 0 ||
				s->rmc.hh < 0 || s->rmc.mm < 0 || s->rmc.ss < 0) {
			date_valid = false;
		}

		bool gga_updated = false;
		if (s->gga_cnt != gga_cnt_last) {
			gga_updated = true;
			gga_cnt_last = s->gga_cnt;
		}

		bool rmc_updated = false;
		if (s->rmc_cnt != rmc_cnt_last) {
			rmc_updated = true;
			rmc_cnt_last = s->rmc_cnt;
		}

		if (m_field_num > 0 && !f_log) {
			if ((m_append_gnss || m_append_gnss_time) && !date_valid) {
				vTaskDelay(configTICK_RATE_HZ / 100);
				continue;
			}

			if (date_valid) {
				char path[200];
				sprintf(path,
						"/sdcard/log_can/date/%02d-%02d-%02d %02d-%02d-%02d.csv",
						s->rmc.yy, s->rmc.mo, s->rmc.dd, s->rmc.hh, s->rmc.mm, s->rmc.ss);
				f_log = fopen(path, "w");
			} else {
				char path[200];
				for (int i = 0;i < 999;i++) {
					sprintf(path, "/sdcard/log_can/no_date/log_%03d.csv", i);
					if (access(path, F_OK) != 0) {
						f_log = fopen(path, "w");
						break;
					}
				}
			}

			if (f_log) {
				// To get the first sample
				gga_updated = true;
				rmc_updated = true;

				for (int i = 0;i < m_field_num;i++) {
					print_header((log_header*)&m_headers[i], f_log);
					if (i == (m_field_num - 1)) {
						if (m_append_time || m_append_gnss_time || m_append_gnss) {
							fprintf(f_log, ";");
						}

						if (m_append_time) {
							print_header(&m_header_ts, f_log);
							if (m_append_gnss_time || m_append_gnss) {
								fprintf(f_log, ";");
							}
						}

						if (m_append_gnss_time) {
							print_header(&m_header_ts_gnss, f_log);
							if (m_append_gnss) {
								fprintf(f_log, ";");
							}
						}

						if (m_append_gnss) {
							print_header(&m_header_lat, f_log);
							fprintf(f_log, ";");

							print_header(&m_header_lon, f_log);
							fprintf(f_log, ";");

							print_header(&m_header_alt, f_log);
							fprintf(f_log, ";");

							print_header(&m_header_hacc, f_log);
							fprintf(f_log, ";");

							print_header(&m_header_hvel, f_log);
						}

						fprintf(f_log, "\n");
					} else {
						fprintf(f_log, ";");
					}
				}
			}
		}

		if (m_field_num <= 0 && f_log) {
			fclose(f_log);
			f_log = 0;
		}

		if (f_log) {
			for (int i = 0;i < m_field_num;i++) {
				log_header *h = (log_header*)&m_headers[i];
				if (h->updated) {
					fprintf(f_log, "%.*f", h->precision, h->value);
					h->updated = false;
				}
				if (i == (m_field_num - 1)) {
					if (m_append_time || m_append_gnss_time || m_append_gnss) {
						fprintf(f_log, ";");
					}

					if (m_append_time) {
						fprintf(f_log, "%.3f", (float)utils_ms_today() / 1000.0);
						if (m_append_gnss_time || m_append_gnss) {
							fprintf(f_log, ";");
						}
					}

					if (m_append_gnss_time) {
						if (gga_updated) {
							fprintf(f_log, "%.3f", (float)s->gga.ms_today / 1000.0);
						}
						if (m_append_gnss) {
							fprintf(f_log, ";");
						}
					}

					if (m_append_gnss) {
						if (gga_updated) {
							fprintf(f_log, "%.8f", s->gga.lat);
						}
						fprintf(f_log, ";");

						if (gga_updated) {
							fprintf(f_log, "%.8f", s->gga.lon);
						}
						fprintf(f_log, ";");

						if (gga_updated) {
							fprintf(f_log, "%.2f", s->gga.height);
						}
						fprintf(f_log, ";");

						if (gga_updated) {
							fprintf(f_log, "%.2f", s->gga.h_dop * 4.0);
						}
						fprintf(f_log, ";");

						if (rmc_updated) {
							fprintf(f_log, "%.2f", s->rmc.speed * 3.6);
						}
					}

					fprintf(f_log, "\n");
				} else {
					fprintf(f_log, ";");
				}
			}

			if (UTILS_AGE_S(tick_last_fsync) > 2.0) {
				tick_last_fsync = xTaskGetTickCount();
				fsync(fileno(f_log));
			}
		}

		if (m_rate_hz < 0.1) {
			m_rate_hz = 10.0;
		}

		float task_time = (float)(utils_ms_tot() - ms_last) / 1000.0;
		int sleep_time = (int)((float)configTICK_RATE_HZ  * ((1.0 / m_rate_hz) - task_time));
		if (sleep_time < 0) {
			sleep_time = 1;
		}
		vTaskDelay(sleep_time);
		ms_last = utils_ms_tot();
	}
}

bool log_mount_card(int pin_mosi, int pin_miso, int pin_sck, int pin_cs, int freq) {
	esp_err_t ret;

	esp_vfs_fat_sdmmc_mount_config_t mount_config = {
			.format_if_mount_failed = true,
			.max_files = 5,
			.allocation_unit_size = 0
	};

	m_host.max_freq_khz = freq;

	spi_bus_config_t bus_cfg = {
			.mosi_io_num = pin_mosi,
			.miso_io_num = pin_miso,
			.sclk_io_num = pin_sck,
			.quadwp_io_num = -1,
			.quadhd_io_num = -1,
			.max_transfer_sz = 4092,
	};

	log_unmount_card();

	ret = spi_bus_initialize(m_host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
	if (ret != ESP_OK) {
		return false;
	}

	sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
	slot_config.gpio_cs = pin_cs;
	slot_config.host_id = m_host.slot;

	ret = esp_vfs_fat_sdspi_mount("/sdcard", &m_host, &slot_config, &mount_config, &m_card);

	if (ret != ESP_OK) {
		return false;
	}

	return true;
}

void log_unmount_card(void) {
	if (m_card) {
		esp_vfs_fat_sdcard_unmount("/sdcard", m_card);
		m_card = 0;
	}

	spi_bus_free(m_host.slot);
}

bool log_init(void) {
	for (int i = 0;i < LOG_MAX_FIELDS;i++) {
		sprintf((char*)m_headers[i].key, "key_h%d", i);
		sprintf((char*)m_headers[i].name, "name_h%d", i);
		strcpy((char*)m_headers[i].unit, "");
		m_headers[i].precision = 2;
		m_headers[i].is_relative = false;
		m_headers[i].is_timestamp = false;
		m_headers[i].value = 0.0;
		m_headers[i].updated = false;
	}

	// Special headers
	strcpy(m_header_ts.key, "t_day");
	strcpy(m_header_ts.name, "Time");
	strcpy(m_header_ts.unit, "s");
	m_header_ts.precision = 3;
	m_header_ts.is_relative = false;
	m_header_ts.is_timestamp = true;
	m_header_ts.value = 0.0;
	m_header_ts.updated = false;

	strcpy(m_header_ts_gnss.key, "t_day_pos");
	strcpy(m_header_ts_gnss.name, "Time GNSS");
	strcpy(m_header_ts_gnss.unit, "s");
	m_header_ts_gnss.precision = 3;
	m_header_ts_gnss.is_relative = false;
	m_header_ts_gnss.is_timestamp = true;
	m_header_ts_gnss.value = 0.0;
	m_header_ts_gnss.updated = false;

	strcpy(m_header_lat.key, "gnss_lat");
	strcpy(m_header_lat.name, "Latitude");
	strcpy(m_header_lat.unit, "deg");
	m_header_lat.precision = 7;
	m_header_lat.is_relative = false;
	m_header_lat.is_timestamp = false;
	m_header_lat.value = 0.0;
	m_header_lat.updated = false;

	strcpy(m_header_lon.key, "gnss_lon");
	strcpy(m_header_lon.name, "Longitude");
	strcpy(m_header_lon.unit, "deg");
	m_header_lon.precision = 7;
	m_header_lon.is_relative = false;
	m_header_lon.is_timestamp = false;
	m_header_lon.value = 0.0;
	m_header_lon.updated = false;

	strcpy(m_header_alt.key, "gnss_alt");
	strcpy(m_header_alt.name, "Altitude");
	strcpy(m_header_alt.unit, "m");
	m_header_alt.precision = 2;
	m_header_alt.is_relative = false;
	m_header_alt.is_timestamp = false;
	m_header_alt.value = 0.0;
	m_header_alt.updated = false;

	strcpy(m_header_hacc.key, "gnss_h_acc");
	strcpy(m_header_hacc.name, "H. Accuracy GNSS");
	strcpy(m_header_hacc.unit, "m");
	m_header_hacc.precision = 2;
	m_header_hacc.is_relative = false;
	m_header_hacc.is_timestamp = false;
	m_header_hacc.value = 0.0;
	m_header_hacc.updated = false;

	strcpy(m_header_hvel.key, "gnss_h_vel");
	strcpy(m_header_hvel.name, "H. Speed GNSS");
	strcpy(m_header_hvel.unit, "km/h");
	m_header_hvel.precision = 2;
	m_header_hvel.is_relative = false;
	m_header_hvel.is_timestamp = false;
	m_header_hvel.value = 0.0;
	m_header_hvel.updated = false;

	xTaskCreatePinnedToCore(log_task, "log", 3072, NULL, 8, NULL, tskNO_AFFINITY);

	return true;
}

void log_process_packet(unsigned char *data, unsigned int len) {
	COMM_PACKET_ID packet_id = data[0];
	data++;
	len--;

	switch (packet_id) {
	case COMM_LOG_START: {
		if (m_field_num > 0) {
			break;
		}

		mkdir("/sdcard/log_can", 0775);
		mkdir("/sdcard/log_can/date", 0775);
		mkdir("/sdcard/log_can/no_date", 0775);

		int32_t ind = 0;
		m_field_num = buffer_get_int16(data, &ind);
		m_rate_hz = buffer_get_float32_auto(data, &ind);
		m_append_time = data[ind++];
		m_append_gnss = data[ind++];
		m_append_gnss_time = data[ind++];
	} break;

	case COMM_LOG_STOP: {
		m_field_num = 0;
	} break;

	case COMM_LOG_CONFIG_FIELD: {
		int32_t ind = 0;
		int field_ind = buffer_get_int16(data, &ind);
		if (field_ind >= 0 && field_ind < LOG_MAX_FIELDS) {
			strncpy((char*)m_headers[field_ind].key, (char*)data + ind, sizeof(m_headers[field_ind].key));
			m_headers[field_ind].key[sizeof(m_headers[field_ind].key) - 1] = '\0';
			ind += strlen((char*)(data + ind)) + 1;

			strncpy((char*)m_headers[field_ind].name, (char*)data + ind, sizeof(m_headers[field_ind].name));
			m_headers[field_ind].name[sizeof(m_headers[field_ind].name) - 1] = '\0';
			ind += strlen((char*)(data + ind)) + 1;

			strncpy((char*)m_headers[field_ind].unit, (char*)data + ind, sizeof(m_headers[field_ind].unit));
			m_headers[field_ind].unit[sizeof(m_headers[field_ind].unit) - 1] = '\0';
			ind += strlen((char*)(data + ind)) + 1;

			m_headers[field_ind].precision = data[ind++];
			m_headers[field_ind].is_relative = data[ind++];
			m_headers[field_ind].is_timestamp = data[ind++];
		}
	} break;

	case COMM_LOG_DATA_F32: {
		int32_t ind = 0;
		int field_ind = buffer_get_int16(data, &ind);

		if (field_ind < 0) {
			break;
		}

		while (field_ind < LOG_MAX_FIELDS && ind < len) {
			m_headers[field_ind].value = buffer_get_float32_auto(data, &ind);
			m_headers[field_ind].updated = true;
			field_ind++;
		}
	} break;

	case COMM_LOG_DATA_F64: {
		int32_t ind = 0;
		int field_ind = buffer_get_int16(data, &ind);

		if (field_ind < 0) {
			break;
		}

		while (field_ind < LOG_MAX_FIELDS && ind < len) {
			m_headers[field_ind].value = buffer_get_float64_auto(data, &ind);
			m_headers[field_ind].updated = true;
			field_ind++;
		}
	} break;

	default:
		break;
	}
}
