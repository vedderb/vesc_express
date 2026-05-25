#include "ublox.h"

bool ublox_init(bool print, uint16_t rate_ms, int num_uart, int pin_rx, int pin_tx) {
	(void)print;
	(void)rate_ms;
	(void)num_uart;
	(void)pin_rx;
	(void)pin_tx;
	return false;
}

void ublox_stop(int num_uart) {
	(void)num_uart;
}

bool ublox_init_ok(void) {
	return false;
}

void ublox_send(unsigned char *data, unsigned int len) {
	(void)data;
	(void)len;
}

void ublox_set_rx_callback_nav_sol(void(*func)(ubx_nav_sol *sol)) {
	(void)func;
}

void ublox_set_rx_callback_relposned(void(*func)(ubx_nav_relposned *pos)) {
	(void)func;
}

void ublox_set_rx_callback_rawx(void(*func)(ubx_rxm_rawx *rawx)) {
	(void)func;
}

void ublox_set_rx_callback_svin(void(*func)(ubx_nav_svin *svin)) {
	(void)func;
}

void ublox_set_rx_callback_nav_sat(void(*func)(ubx_nav_sat *sat)) {
	(void)func;
}

void ublox_set_rx_callback_cfg_gnss(void(*func)(ubx_cfg_gnss *gnss)) {
	(void)func;
}

void ublox_poll(uint8_t msg_class, uint8_t id) {
	(void)msg_class;
	(void)id;
}

int ublox_cfg_prt_uart(ubx_cfg_prt_uart *cfg) {
	(void)cfg;
	return -1;
}

int ublox_cfg_tmode3(ubx_cfg_tmode3 *cfg) {
	(void)cfg;
	return -1;
}

int ublox_cfg_msg(uint8_t msg_class, uint8_t id, uint8_t rate) {
	(void)msg_class;
	(void)id;
	(void)rate;
	return -1;
}

int ublox_cfg_rate(uint16_t meas_rate_ms, uint16_t nav_rate_ms, uint16_t time_ref) {
	(void)meas_rate_ms;
	(void)nav_rate_ms;
	(void)time_ref;
	return -1;
}

int ublox_cfg_cfg(ubx_cfg_cfg *cfg) {
	(void)cfg;
	return -1;
}

int ublox_cfg_nav5(ubx_cfg_nav5 *cfg) {
	(void)cfg;
	return -1;
}

int ublox_cfg_tp5(ubx_cfg_tp5 *cfg) {
	(void)cfg;
	return -1;
}

int ublox_cfg_gnss(ubx_cfg_gnss *gnss) {
	(void)gnss;
	return -1;
}

int ublox_cfg_nmea(ubx_cfg_nmea *nmea) {
	(void)nmea;
	return -1;
}

int ublox_cfg_valset(unsigned char *values, int len,
		bool ram, bool bbr, bool flash) {
	(void)values;
	(void)len;
	(void)ram;
	(void)bbr;
	(void)flash;
	return -1;
}

void ublox_cfg_append_enable_gps(unsigned char *buffer, int *ind,
		bool en, bool en_l1, bool en_l2) {
	(void)buffer;
	(void)ind;
	(void)en;
	(void)en_l1;
	(void)en_l2;
}

void ublox_cfg_append_enable_gal(unsigned char *buffer, int *ind,
		bool en, bool en_e1, bool en_e5b) {
	(void)buffer;
	(void)ind;
	(void)en;
	(void)en_e1;
	(void)en_e5b;
}

void ublox_cfg_append_enable_bds(unsigned char *buffer, int *ind,
		bool en, bool en_b1, bool en_b2) {
	(void)buffer;
	(void)ind;
	(void)en;
	(void)en_b1;
	(void)en_b2;
}

void ublox_cfg_append_enable_glo(unsigned char *buffer, int *ind,
		bool en, bool en_l1, bool en_l2) {
	(void)buffer;
	(void)ind;
	(void)en;
	(void)en_l1;
	(void)en_l2;
}

void ublox_cfg_append_uart1_baud(unsigned char *buffer, int *ind, uint32_t baud) {
	(void)buffer;
	(void)ind;
	(void)baud;
}

void ublox_cfg_append_rate(unsigned char *buffer, int *ind, uint16_t meas_ms, uint16_t nav) {
	(void)buffer;
	(void)ind;
	(void)meas_ms;
	(void)nav;
}

void ublox_cfg_append_u1(unsigned char *buffer, int *ind, uint32_t key, uint8_t val) {
	(void)buffer;
	(void)ind;
	(void)key;
	(void)val;
}
