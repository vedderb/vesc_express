/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAIN_NMEA_H_
#define MAIN_NMEA_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	double lat;
	double lon;
	double height;
	int ms_today;
	int n_sat;
	int fix_type;
	float h_dop;
	float diff_age;
	uint32_t update_time;
} nmea_gga_info_t;

typedef struct {
	int prn;
	float elevation;
	float azimuth;
	float snr;
	bool lock;
	float base_snr;
	bool base_lock;
	bool local_lock;
} nmea_gsv_sat_t;

typedef struct {
	int sat_num;
	int sentences;
	int sat_last;
	int sat_num_base;
	nmea_gsv_sat_t sats[32];
	uint32_t update_time;
} nmea_gsv_info_t;

typedef struct {
	// UTC Time
	int hh; // Hours
	int mm; // Minutes
	int ss; // Seconds
	int ms; // Milliseconds
	int yy; // Year
	int mo; // Month
	int dd; // Day
	float speed; // Ground speed, meters per second
	uint32_t update_time;
} nmea_rmc_info_t;

typedef struct {
	int gga_cnt;
	int gsv_gp_cnt;
	int gsv_gl_cnt;
	int rmc_cnt;
	nmea_gga_info_t gga;
	nmea_gsv_info_t gsv;
	nmea_rmc_info_t rmc;
} nmea_state_t;

// Functions
void nmea_init(void);
nmea_state_t* nmea_get_state(void);
const char* nmea_fix_type(void);
bool nmea_decode_string(const char *data);
int nmea_decode_gga(const char *data, nmea_gga_info_t *gga);
int nmea_decode_gsv(const char *system_str, const char *data, nmea_gsv_info_t *gsv_info);
void nmea_sync_gsv_info(nmea_gsv_info_t *old_info, nmea_gsv_info_t *new_info);
int nmea_decode_rmc(const char *data, nmea_rmc_info_t *rmc);

#endif /* MAIN_NMEA_H_ */
