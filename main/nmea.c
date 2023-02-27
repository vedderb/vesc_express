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

#include "nmea.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// For double precision literals
#define D(x) 						((double)x##L)

// Private variables
static nmea_state_t m_state = {0};

void nmea_init(void) {
	m_state.rmc.hh = -1;
	m_state.rmc.mm = -1;
	m_state.rmc.ss = -1;
	m_state.rmc.ms = -1;
	m_state.rmc.yy = -1;
	m_state.rmc.mo = -1;
	m_state.rmc.dd = -1;
}

nmea_state_t* nmea_get_state(void) {
	return &m_state;
}

const char* nmea_fix_type(void) {
	switch (m_state.gga.fix_type) {
		case 0: return "No fix";
		case 1: return "Autonomous GNSS";
		case 2: return "Differential GNSS";
		case 4: return "RTK fix";
		case 5: return "RTK float";
		case 6: return "Dead reckoning";
		default: return "Unknown";
	}
}

bool nmea_decode_string(const char *data) {
	bool ok = false;

	static nmea_gsv_info_t gpgsv;
	static nmea_gsv_info_t glgsv;

	int gga_res = nmea_decode_gga(data, &(m_state.gga));
	int gpgsv_res = nmea_decode_gsv("GP", data, &gpgsv);
	int glgsv_res = nmea_decode_gsv("GL", data, &glgsv);
	int rmc_res = nmea_decode_rmc(data, &(m_state.rmc));

	if (gga_res >= 0) {
		m_state.gga.update_time = xTaskGetTickCount();
		m_state.gga_cnt++;
		ok = true;
	}

	if (gpgsv_res == 1) {
		nmea_sync_gsv_info(&(m_state.gsv), &gpgsv);
		m_state.gsv.update_time = xTaskGetTickCount();
		m_state.gsv_gp_cnt++;
		ok = true;
	}

	if (glgsv_res == 1) {
		nmea_sync_gsv_info(&(m_state.gsv), &glgsv);
		m_state.gsv.update_time = xTaskGetTickCount();
		m_state.gsv_gl_cnt++;
		ok = true;
	}

	if (rmc_res >= 0) {
		m_state.rmc.update_time = xTaskGetTickCount();
		m_state.rmc_cnt++;
		ok = true;
	}

	return ok;
}

static bool nmea_parse_val(char *str, double *val) {
	int ind = -1;
	int len = strlen(str);
	bool retval = false;

	for (int i = 2;i < len;i++) {
		if (str[i] == '.') {
			ind = i - 2;
			break;
		}
	}

	if (ind >= 0) {
		char a[len + 1];
		memcpy(a, str, ind);
		a[ind] = ' ';
		memcpy(a + ind + 1, str + ind, len - ind);

		double l1, l2;
		if (sscanf(a, "%lf %lf", &l1, &l2) == 2) {
			*val = l1 + l2 / D(60.0);
			retval = true;
		}
	}

	return retval;
}

/**
 * Decode NMEA GGA message.
 *
 * @param data
 * NMEA string.
 *
 * @param gga
 * GGA struct to fill.
 *
 * @return
 * -1: Type is not GGA
 * >= 0: Number of decoded fields.
 */
int nmea_decode_gga(const char *data, nmea_gga_info_t *gga) {
	int ms = gga->ms_today;
	double lat = gga->lat;
	double lon = gga->lon;
	double height = gga->height;
	int fix_type = gga->fix_type;
	int sats = gga->n_sat;
	float hdop = gga->h_dop;
	float diff_age = gga->diff_age;

	int dec_fields = 0;

	bool found = false;
	int len = strlen(data);
	char *str_tmp = malloc(len + 1);

	for (int i = 0;i < 10;i++) {
		if ((i + 5) >= len) {
			break;
		}

		if (	data[i] == 'G' &&
				data[i + 1] == 'G' &&
				data[i + 2] == 'A' &&
				data[i + 3] == ',') {
			found = true;
			strcpy(str_tmp, data + i + 4);
			break;
		}
	}

	if (found) {
		char *token, *str;
		int ind = 0;

		str = str_tmp;
		token = strsep(&str, ",");

		while (token != 0) {
			if (token[0] == '*') {
				break;
			}

			switch (ind) {
			case 0: {
				// Time
				int h, m, s, ds;
				dec_fields++;

				if (sscanf(token, "%02d%02d%02d.%d", &h, &m, &s, &ds) == 4) {
					ms = h * 60 * 60 * 1000;
					ms += m * 60 * 1000;
					ms += s * 1000;
					ms += ds * 10;
				} else {
					ms = -1;
				}
			} break;

			case 1: {
				// Latitude
				if (nmea_parse_val(token, &lat)) {
					dec_fields++;
				}
			} break;

			case 2:
				// Latitude direction
				dec_fields++;
				if (*token == 'S' || *token == 's') {
					lat = -lat;
				}
				break;

			case 3: {
				// Longitude
				if (nmea_parse_val(token, &lon)) {
					dec_fields++;
				}
			} break;

			case 4:
				// Longitude direction
				dec_fields++;
				if (*token == 'W' || *token == 'w') {
					lon = -lon;
				}
				break;

			case 5:
				// Fix type
				dec_fields++;
				if (sscanf(token, "%d", &fix_type) != 1) {
					fix_type = 0;
				}
				break;

			case 6:
				// Satellites
				if (sscanf(token, "%d", &sats) == 1) {
					dec_fields++;
				}
				break;

			case 7:
				// hdop
				if (sscanf(token, "%f", &hdop) == 1) {
					dec_fields++;
				}
				break;

			case 8:
				// Altitude
				if (sscanf(token, "%lf", &height) == 1) {
					dec_fields++;
				}
				break;

			case 10: {
				// Altitude 2
				double h2 = 0.0;
				dec_fields++;
				if (sscanf(token, "%lf", &h2) != 1) {
					h2 = 0.0;
				}

				height += h2;
			} break;

			case 12: {
				// Correction age
				dec_fields++;
				if (sscanf(token, "%f", &diff_age) != 1) {
					diff_age = -1.0;
				}
			} break;

			default:
				break;
			}

			token = strsep(&str, ",");
			ind++;
		}
	} else {
		dec_fields = -1;
	}

	free(str_tmp);

	// 64-bit writes are not atomic
	portDISABLE_INTERRUPTS();
	gga->lat = lat;
	gga->lon = lon;
	portENABLE_INTERRUPTS();

	gga->height = height;
	gga->fix_type = fix_type;
	gga->n_sat = sats;
	gga->ms_today = ms;
	gga->h_dop = hdop;
	gga->diff_age = diff_age;

	return dec_fields;
}

/**
 * Decode NMEA GSV message.
 *
 * @param system_str
 * Satellite system string:
 * GP: GPS
 * GN: GLONASS
 * GA: GALILEO
 *
 * @param data
 * NMEA string.
 *
 * @param gsv_info
 * GSV struct to fill.
 *
 * @return
 * -2: Unknown error
 * -1: Wrong type (not gsv)
 * 0: Decode ok, waiting for more sentences
 * 1: All sentences decoded, data ready to be used
 */
int nmea_decode_gsv(const char *system_str, const char *data, nmea_gsv_info_t *gsv_info) {
	int retval = -2;

	bool found = false;
	int len = strlen(data);
	char *str_tmp = malloc(len + 1);

	for (int i = 0;i < 10;i++) {
		if ((i + 7) >= len) {
			break;
		}

		if (    data[i] == system_str[0] &&
				data[i + 1] == system_str[1] &&
				data[i + 2] == 'G' &&
				data[i + 3] == 'S' &&
				data[i + 4] == 'V' &&
				data[i + 5] == ',') {
			found = true;
			strcpy(str_tmp, data + i + 6);
			break;
		}
	}

	if (found) {
		char *token, *str;
		int ind = 0;

		str = str_tmp;
		token = strsep(&str, ",");

		while (token !=0) {
			if (token[0] == '*') {
				break;
			}

			switch (ind) {
			case 0: {
				// Number of sentences
				if (sscanf(token, "%d", &(gsv_info->sentences)) != 1) {
					gsv_info->sentences = 0;
				}
			} break;

			case 1: {
				// Sentence now
				int sentence = 0;
				if (sscanf(token, "%d", &sentence) != 1) {
					sentence = 0;
				}

				if (sentence == 1) {
					gsv_info->sat_last = 0;
				}
			} break;

			case 2: {
				// Sats
				if (sscanf(token, "%d", &(gsv_info->sat_num)) != 1) {
					gsv_info->sat_num = 0;
				}
			} break;

			case 3: {
				// PRN
				if (gsv_info->sat_last < 32) {
					int prn = 0;
					sscanf(token, "%d", &prn);
					gsv_info->sats[gsv_info->sat_last].prn = prn;
				}
			} break;

			case 4: {
				// Elevation
				if (gsv_info->sat_last < 32) {
					float elev = 0.0;
					sscanf(token, "%f", &elev);
					gsv_info->sats[gsv_info->sat_last].elevation = elev;
				}
			} break;

			case 5: {
				// Azimuth
				if (gsv_info->sat_last < 32) {
					float azimuth = 0.0;
					sscanf(token, "%f", &azimuth);
					gsv_info->sats[gsv_info->sat_last].azimuth = azimuth;
				}
			} break;

			case 6: {
				// SNR
				if (gsv_info->sat_last < 32) {
					float snr = 0.0;
					sscanf(token, "%f", &snr);
					gsv_info->sats[gsv_info->sat_last].snr = snr;
					gsv_info->sat_last++;
					ind = 2;
				}
			} break;

			default:
				break;
			}

			token = strsep(&str, ",");
			ind++;
		}

		if (gsv_info->sat_last == gsv_info->sat_num) {
			retval = 1;
		} else {
			retval = 0;
		}
	} else {
		retval = -1;
	}

	free(str_tmp);

	return retval;
}

/**
 * Synchronize nmea gsv info structs.
 *
 * @param old_info
 * The info struct to update.
 *
 * @param new_info
 * The new info struct to take data from.
 */
void nmea_sync_gsv_info(nmea_gsv_info_t *old_info, nmea_gsv_info_t *new_info) {
	for (int i = 0;i < new_info->sat_num;i++) {
		for (int j = 0;j < old_info->sat_num;j++) {
			if (new_info->sats[i].prn == old_info->sats[j].prn) {
				new_info->sats[i].base_lock = old_info->sats[j].base_lock;
				new_info->sats[i].base_snr = old_info->sats[j].base_snr;
				new_info->sats[i].local_lock = old_info->sats[j].local_lock;
			}
		}

		old_info->sats[i] = new_info->sats[i];
	}

	old_info->sentences = new_info->sentences;
	old_info->sat_num = new_info->sat_num;
	old_info->sat_last = new_info->sat_last;
}

/**
 * Decode NMEA RMC message.
 *
 * @param data
 * NMEA string.
 *
 * @param rmc
 * RMC struct to fill.
 *
 * @return
 * -1: Type is not RMC
 * >= 0: Number of decoded fields.
 */
int nmea_decode_rmc(const char *data, nmea_rmc_info_t *rmc) {
	int hh = rmc->hh;
	int mm = rmc->mm;
	int ss = rmc->ss;
	int ms = rmc->ms;
	int yy = rmc->yy;
	int mo = rmc->mo;
	int dd = rmc->dd;
	float speed = rmc->speed;

	int dec_fields = 0;

	bool found = false;
	int len = strlen(data);
	char *str_tmp = malloc(len + 1);

	for (int i = 0;i < 10;i++) {
		if ((i + 5) >= len) {
			break;
		}

		if (    data[i] == 'R' &&
				data[i + 1] == 'M' &&
				data[i + 2] == 'C' &&
				data[i + 3] == ',') {
			found = true;
			strcpy(str_tmp, data + i + 4);
			break;
		}
	}

	if (found) {
		char *token, *str;
		int ind = 0;

		str = str_tmp;
		token = strsep(&str, ",");

		while (token != 0) {
			if (token[0] == '*') {
				break;
			}

			switch (ind) {
			case 0: {
				// Time
				dec_fields++;

				if (sscanf(token, "%02d%02d%02d.%d", &hh, &mm, &ss, &ms) == 4) {
					ms *= 10;
				}
			} break;

			case 6: {
				// Speed
				sscanf(token, "%f", &speed);
				speed *= 0.51444; // Knots to meters per second
			} break;

			case 8: {
				// Date
				dec_fields++;

				if (sscanf(token, "%02d%02d%02d", &dd, &mo, &yy) == 3) {
					yy += 2000;
				}
			} break;

			default:
				break;
			}

			token = strsep(&str, ",");
			ind++;
		}
	} else {
		dec_fields = -1;
	}

	rmc->hh = hh;
	rmc->mm = mm;
	rmc->ss = ss;
	rmc->ms = ms;
	rmc->yy = yy;
	rmc->mo = mo;
	rmc->dd = dd;
	rmc->speed = speed;

	free(str_tmp);

	return dec_fields;
}
