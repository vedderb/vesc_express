#include "comm_ble.h"

void comm_ble_init(void) {
}

bool comm_ble_is_connected(void) {
	return false;
}

int comm_ble_mtu_now(void) {
	return 0;
}

void comm_ble_send_packet(unsigned char *data, unsigned int len) {
	(void)data;
	(void)len;
}
