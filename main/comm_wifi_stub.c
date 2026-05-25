#include "comm_wifi.h"

void comm_wifi_init(void) {
}

WIFI_MODE comm_wifi_get_mode(void) {
	return WIFI_MODE_DISABLED;
}

esp_ip4_addr_t comm_wifi_get_ip(void) {
	esp_ip4_addr_t ip = {0};
	return ip;
}

esp_ip4_addr_t comm_wifi_get_ip_client(void) {
	esp_ip4_addr_t ip = {0};
	return ip;
}

bool comm_wifi_is_client_connected(void) {
	return false;
}

bool comm_wifi_is_connected_hub(void) {
	return false;
}

bool comm_wifi_is_connecting(void) {
	return false;
}

bool comm_wifi_is_connected(void) {
	return false;
}

void comm_wifi_disconnect(void) {
}

bool comm_wifi_change_network(const char *ssid, const char *password) {
	(void)ssid;
	(void)password;
	return false;
}

bool comm_wifi_reconnect_network(void) {
	return false;
}

bool comm_wifi_disconnect_network(void) {
	return false;
}

bool comm_wifi_set_auto_reconnect(bool should_reconnect) {
	(void)should_reconnect;
	return false;
}

bool comm_wifi_get_auto_reconnect(void) {
	return false;
}

void comm_wifi_set_event_listener(comm_wifi_event_cb_t handler) {
	(void)handler;
}

void comm_wifi_send_packet_local(unsigned char *data, unsigned int len) {
	(void)data;
	(void)len;
}

void comm_wifi_send_packet_hub(unsigned char *data, unsigned int len) {
	(void)data;
	(void)len;
}

void comm_wifi_send_raw_local(unsigned char *buffer, unsigned int len) {
	(void)buffer;
	(void)len;
}

void comm_wifi_send_raw_hub(unsigned char *buffer, unsigned int len) {
	(void)buffer;
	(void)len;
}
