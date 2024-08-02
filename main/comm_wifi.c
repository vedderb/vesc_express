/*
	Copyright 2022 - 2023 Benjamin Vedder    benjamin@vedder.se
	Copyright 2023 Rasmus Söderhielm         rasmus.soderhielm@gmail.com
	

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

#include "comm_wifi.h"
#include "conf_general.h"
#include "utils.h"
#include "main.h"
#include "packet.h"
#include "commands.h"
#include "datatypes.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/netdb.h"

#define WIFI_CONNECTED_BIT		BIT0
#define WIFI_FAIL_BIT			BIT1

static EventGroupHandle_t s_wifi_event_group;
static esp_ip4_addr_t ip = {0};
static bool is_connecting = false;
static bool is_connected = false;
static bool wifi_reconnect_disabled = true;
static bool wifi_auto_reconnect = true;
static WIFI_MODE wifi_mode = WIFI_MODE_DISABLED;
static volatile bool wifi_config_changed = false;
static wifi_config_t wifi_config = {0};

static comm_wifi_event_cb_t event_listener = NULL;

typedef struct {
	PACKET_STATE_t *packet;
	int socket;
	esp_ip4_addr_t ip_client;
} comm_state;

static comm_state comm_local = {.socket = -1, .ip_client = {0}};
static comm_state comm_hub = {.socket = -1, .ip_client = {0}};

// Used for logging
__attribute__((unused))
static const char *wifi_reason_to_str(wifi_err_reason_t reason) {
	switch (reason) {
		case WIFI_REASON_UNSPECIFIED:
			return "UNSPECIFIED";
		case WIFI_REASON_AUTH_EXPIRE:
			return "AUTH_EXPIRE";
		case WIFI_REASON_AUTH_LEAVE:
			return "AUTH_LEAVE";
		case WIFI_REASON_ASSOC_EXPIRE:
			return "ASSOC_EXPIRE";
		case WIFI_REASON_ASSOC_TOOMANY:
			return "ASSOC_TOOMANY";
		case WIFI_REASON_NOT_AUTHED:
			return "NOT_AUTHED";
		case WIFI_REASON_NOT_ASSOCED:
			return "NOT_ASSOCED";
		case WIFI_REASON_ASSOC_LEAVE:
			return "ASSOC_LEAVE";
		case WIFI_REASON_ASSOC_NOT_AUTHED:
			return "ASSOC_NOT_AUTHED";
		case WIFI_REASON_DISASSOC_PWRCAP_BAD:
			return "DISASSOC_PWRCAP_BAD";
		case WIFI_REASON_DISASSOC_SUPCHAN_BAD:
			return "DISASSOC_SUPCHAN_BAD";
		case WIFI_REASON_BSS_TRANSITION_DISASSOC:
			return "BSS_TRANSITION_DISASSOC";
		case WIFI_REASON_IE_INVALID:
			return "IE_INVALID";
		case WIFI_REASON_MIC_FAILURE:
			return "MIC_FAILURE";
		case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
			return "4WAY_HANDSHAKE_TIMEOUT";
		case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:
			return "GROUP_KEY_UPDATE_TIMEOUT";
		case WIFI_REASON_IE_IN_4WAY_DIFFERS:
			return "IE_IN_4WAY_DIFFERS";
		case WIFI_REASON_GROUP_CIPHER_INVALID:
			return "GROUP_CIPHER_INVALID";
		case WIFI_REASON_PAIRWISE_CIPHER_INVALID:
			return "PAIRWISE_CIPHER_INVALID";
		case WIFI_REASON_AKMP_INVALID:
			return "AKMP_INVALID";
		case WIFI_REASON_UNSUPP_RSN_IE_VERSION:
			return "UNSUPP_RSN_IE_VERSION";
		case WIFI_REASON_INVALID_RSN_IE_CAP:
			return "INVALID_RSN_IE_CAP";
		case WIFI_REASON_802_1X_AUTH_FAILED:
			return "802_1X_AUTH_FAILED";
		case WIFI_REASON_CIPHER_SUITE_REJECTED:
			return "CIPHER_SUITE_REJECTED";
		case WIFI_REASON_TDLS_PEER_UNREACHABLE:
			return "TDLS_PEER_UNREACHABLE";
		case WIFI_REASON_TDLS_UNSPECIFIED:
			return "TDLS_UNSPECIFIED";
		case WIFI_REASON_SSP_REQUESTED_DISASSOC:
			return "SSP_REQUESTED_DISASSOC";
		case WIFI_REASON_NO_SSP_ROAMING_AGREEMENT:
			return "NO_SSP_ROAMING_AGREEMENT";
		case WIFI_REASON_BAD_CIPHER_OR_AKM:
			return "BAD_CIPHER_OR_AKM";
		case WIFI_REASON_NOT_AUTHORIZED_THIS_LOCATION:
			return "NOT_AUTHORIZED_THIS_LOCATION";
		case WIFI_REASON_SERVICE_CHANGE_PERCLUDES_TS:
			return "SERVICE_CHANGE_PERCLUDES_TS";
		case WIFI_REASON_UNSPECIFIED_QOS:
			return "UNSPECIFIED_QOS";
		case WIFI_REASON_NOT_ENOUGH_BANDWIDTH:
			return "NOT_ENOUGH_BANDWIDTH";
		case WIFI_REASON_MISSING_ACKS:
			return "MISSING_ACKS";
		case WIFI_REASON_EXCEEDED_TXOP:
			return "EXCEEDED_TXOP";
		case WIFI_REASON_STA_LEAVING:
			return "STA_LEAVING";
		case WIFI_REASON_END_BA:
			return "END_BA";
		case WIFI_REASON_UNKNOWN_BA:
			return "UNKNOWN_BA";
		case WIFI_REASON_TIMEOUT:
			return "TIMEOUT";
		case WIFI_REASON_PEER_INITIATED:
			return "PEER_INITIATED";
		case WIFI_REASON_AP_INITIATED:
			return "AP_INITIATED";
		case WIFI_REASON_INVALID_FT_ACTION_FRAME_COUNT:
			return "INVALID_FT_ACTION_FRAME_COUNT";
		case WIFI_REASON_INVALID_PMKID:
			return "INVALID_PMKID";
		case WIFI_REASON_INVALID_MDE:
			return "INVALID_MDE";
		case WIFI_REASON_INVALID_FTE:
			return "INVALID_FTE";
		case WIFI_REASON_TRANSMISSION_LINK_ESTABLISH_FAILED:
			return "TRANSMISSION_LINK_ESTABLISH_FAILED";
		case WIFI_REASON_ALTERATIVE_CHANNEL_OCCUPIED:
			return "ALTERATIVE_CHANNEL_OCCUPIED";

		case WIFI_REASON_BEACON_TIMEOUT:
			return "BEACON_TIMEOUT";
		case WIFI_REASON_NO_AP_FOUND:
			return "NO_AP_FOUND";
		case WIFI_REASON_AUTH_FAIL:
			return "AUTH_FAIL";
		case WIFI_REASON_ASSOC_FAIL:
			return "ASSOC_FAIL";
		case WIFI_REASON_HANDSHAKE_TIMEOUT:
			return "HANDSHAKE_TIMEOUT";
		case WIFI_REASON_CONNECTION_FAIL:
			return "CONNECTION_FAIL";
		case WIFI_REASON_AP_TSF_RESET:
			return "AP_TSF_RESET";
		case WIFI_REASON_ROAMING:
			return "ROAMING";
		case WIFI_REASON_ASSOC_COMEBACK_TIME_TOO_LONG:
			return "ASSOC_COMEBACK_TIME_TOO_LONG";
		default:
			return "unknown";
	}
}

static void do_comm(const int sock, comm_state *comm) {
	int len;
	char rx_buffer[128];

	comm->socket = sock;

	do {
		len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

		for (int i = 0;i < len;i++) {
			packet_process_byte(rx_buffer[i], comm->packet);
		}
	} while (len > 0);

	comm->socket = -1;
}

static void set_socket_options(int sock) {
	if (sock < 0) {
		return;
	}

	int keepAlive = 1;
	int keepIdle = 5;
	int keepInterval = 5;
	int keepCount = 3;
	int nodelay = 1;

	setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
	setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
	setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
	setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
	setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(int));
}

static void tcp_task_local(void *arg) {
	for (;;) {
		struct sockaddr_storage dest_addr;
		struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
		dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
		dest_addr_ip4->sin_family = AF_INET;
		dest_addr_ip4->sin_port = htons(65102);

		int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
		int opt = 1;
		setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
		bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
		listen(listen_sock, 1);

		struct sockaddr addr;
		socklen_t addr_len = sizeof(addr);
		int sock = accept(listen_sock, &addr, &addr_len);

		memcpy(&comm_local.ip_client, addr.sa_data + 2, 4);
		set_socket_options(sock);

		do_comm(sock, &comm_local);
		shutdown(sock, 0);
		close(sock);

		shutdown(listen_sock, 0);
		close(listen_sock);
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

static void tcp_task_hub(void *arg) {
	for (;;) {
		ip_addr_t addr;
		{
			err_t result =
				netconn_gethostbyname((char *)backup.config.tcp_hub_url, &addr);

			if (result != ERR_OK) {
				vTaskDelay(1000 / portTICK_PERIOD_MS);
				continue;
			}
		}

		struct sockaddr_in dest_addr =
			create_sockaddr_in(addr, backup.config.tcp_hub_port);

		int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
		int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in));
		if (err == 0) {
			memcpy(&comm_hub.ip_client, &dest_addr.sin_addr.s_addr, 4);
			set_socket_options(sock);

			{
				char buf[60];
				sprintf(buf, "VESC:%s:%s\n", backup.config.tcp_hub_id, backup.config.tcp_hub_pass);
				send(sock, buf, strlen(buf) + 1, 0);
			}
			do_comm(sock, &comm_hub);
		}
		
		shutdown(sock, 0);
		close(sock);
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

/**
 * Broadcast name, IP and port so that VESC Tool can find this device.
 */
static void broadcast_task(void *arg) {
	int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);

	int bc = 1;
	setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bc, sizeof(bc));

	struct sockaddr_in sDestAddr;
	memset(&sDestAddr, 0, sizeof(sDestAddr));
	sDestAddr.sin_family = AF_INET;
	sDestAddr.sin_len = sizeof(sDestAddr);
	sDestAddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	sDestAddr.sin_port = htons(65109);

	for (;;) {
		char sendbuf[50];
		size_t ind = 0;
		if (wifi_mode == WIFI_MODE_ACCESS_POINT) {
			ind += sprintf(sendbuf, "%s::192.168.4.1::65102", backup.config.ble_name) + 1;
		} else {
			ind += sprintf(sendbuf, "%s::" IPSTR "::65102", backup.config.ble_name, IP2STR(&ip)) + 1;
		}

		if (backup.config.use_tcp_local) {
			sendto(sock, sendbuf, ind, 0, (struct sockaddr *)&sDestAddr, sizeof(sDestAddr));
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

static void process_packet_local(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, comm_wifi_send_packet_local);
}

static void process_packet_hub(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, comm_wifi_send_packet_hub);
}

void comm_wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		if (!wifi_reconnect_disabled) {
			is_connecting = true;
			esp_wifi_connect();
		}
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		wifi_event_sta_disconnected_t *data =
			(wifi_event_sta_disconnected_t *)event_data;

		bool is_expected_reason = data->reason == WIFI_REASON_ASSOC_LEAVE
			|| data->reason == WIFI_REASON_AUTH_EXPIRE;
			
		bool will_reconnect = !wifi_reconnect_disabled && (wifi_auto_reconnect || is_expected_reason);

		STORED_LOGF(
			"disconnected, ssid_len: %u, ssid: '%s', reason: '%s' (%u), rssi: "
			"%d, will_reconnect: %s",
			data->ssid_len, data->ssid,
			wifi_reason_to_str(data->reason),
			data->reason, data->rssi, utils_bool_to_str(will_reconnect)
		);

		// We don't cleanup custom sockets here (which are all most likely
		// invalid at this point). Maybe that should change?
		is_connected = false;
		LED_RED_OFF();

		if (will_reconnect) {
			STORED_LOGF("reconnecting to network...");
			is_connecting    = true;
			esp_err_t result = esp_wifi_connect();
			if (result != ESP_OK) {
				STORED_LOGF("esp_wifi_connect failed, result: %d", result);
				is_connecting = false;
			}
		} else {
			is_connecting = false;
		}
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_STOP) {
		// Cleanup custom sockets.
		comm_wifi_disconnect();
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		STORED_LOGF("connected to network! (IP_EVENT_STA_GOT_IP)");
		
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ip = event->ip_info.ip;
		is_connecting = false;
		is_connected = true;
		LED_RED_ON();
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}

	if (event_listener != NULL) {
		event_listener(event_base, event_id, event_data);
	}
}

void comm_wifi_send_packet_local(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, comm_local.packet);
}

void comm_wifi_send_packet_hub(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, comm_hub.packet);
}

#define SEND_RAW_MAX_RETRIES 100

void comm_wifi_send_raw_local(unsigned char *buffer, unsigned int len) {
	if (comm_local.socket < 0) {
		return;
	}

	int error_cnt = 0;

	int to_write = len;
	while (to_write > 0) {
		int written = send(comm_local.socket, buffer + (len - to_write), to_write, 0);
		if (written < 0) {
			error_cnt++;

			if (error_cnt > SEND_RAW_MAX_RETRIES) {
				return;
			}

			vTaskDelay(1);
			continue;
		}

		to_write -= written;
	}
}

void comm_wifi_send_raw_hub(unsigned char *buffer, unsigned int len) {
	if (comm_hub.socket < 0) {
		return;
	}

	int error_cnt = 0;

	int to_write = len;
	while (to_write > 0) {
		int written = send(comm_hub.socket, buffer + (len - to_write), to_write, 0);
		if (written < 0) {
			error_cnt++;

			if (error_cnt > SEND_RAW_MAX_RETRIES) {
				return;
			}

			vTaskDelay(1);
			continue;
		}

		to_write -= written;
	}
}

void comm_wifi_init(void) {
	s_wifi_event_group = xEventGroupCreate();
	esp_netif_init();
	esp_event_loop_create_default();
	
	wifi_mode = backup.config.wifi_mode;

	if (wifi_mode == WIFI_MODE_ACCESS_POINT) {
		esp_netif_create_default_wifi_ap();
	} else {
		esp_netif_create_default_wifi_sta();
	}

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);

	esp_wifi_set_storage(WIFI_STORAGE_RAM);

	if (backup.config.ble_mode == BLE_MODE_DISABLED) {
		esp_wifi_set_ps(WIFI_PS_NONE);
	}

	esp_event_handler_instance_t instance_any_id;
	esp_event_handler_instance_t instance_got_ip;

	esp_event_handler_instance_register(
			WIFI_EVENT,
			ESP_EVENT_ANY_ID,
			&comm_wifi_event_handler,
			NULL,
			&instance_any_id);

	esp_event_handler_instance_register(
			IP_EVENT,
			IP_EVENT_STA_GOT_IP,
			&comm_wifi_event_handler,
			NULL,
			&instance_got_ip);

	esp_wifi_set_mode(WIFI_MODE_APSTA);

	if (wifi_mode == WIFI_MODE_ACCESS_POINT) {
		wifi_config = (wifi_config_t){
			.ap = {
				.ssid = "",
				.ssid_len = strlen((char*)backup.config.wifi_ap_ssid),
				.channel = 1,
				.password = "",
				.max_connection = 4,
				.authmode = WIFI_AUTH_WPA_WPA2_PSK,
				.pmf_cfg = {
					.required = false,
				},
				.ftm_responder = true,
			},
		};

		strcpy((char*)wifi_config.ap.ssid, (char*)backup.config.wifi_ap_ssid);
		strcpy((char*)wifi_config.ap.password, (char*)backup.config.wifi_ap_key);

		esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
	} else {
		wifi_config = (wifi_config_t){
			.sta = {
				.ssid = "",
				.password = "",
				.threshold.authmode = WIFI_AUTH_WEP,
			},
		};

		strcpy((char*)wifi_config.sta.ssid, (char*)backup.config.wifi_sta_ssid);
		strcpy((char*)wifi_config.sta.password, (char*)backup.config.wifi_sta_key);

		esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

		// Enable FTM responder
		esp_wifi_get_config(WIFI_IF_AP, &wifi_config);
		wifi_config.ap.ftm_responder = true;
		esp_wifi_set_config(WIFI_IF_AP, &wifi_config);

		wifi_reconnect_disabled = false;
	}

	esp_wifi_start();

	if (backup.config.use_tcp_local) {
		comm_local.packet = calloc(1, sizeof(PACKET_STATE_t));
		packet_init(comm_wifi_send_raw_local, process_packet_local, comm_local.packet);
		xTaskCreatePinnedToCore(tcp_task_local, "tcp_local", 3500, NULL, 8, NULL, tskNO_AFFINITY);
	}

	if (backup.config.use_tcp_hub) {
		comm_hub.packet = calloc(1, sizeof(PACKET_STATE_t));
		packet_init(comm_wifi_send_raw_hub, process_packet_hub, comm_hub.packet);
		xTaskCreatePinnedToCore(tcp_task_hub, "tcp_hub", 3500, NULL, 8, NULL, tskNO_AFFINITY);
	}

	xTaskCreatePinnedToCore(broadcast_task, "udp_multicast", 2048, NULL, 8, NULL, tskNO_AFFINITY);
}

WIFI_MODE comm_wifi_get_mode(void) {
	return wifi_mode;
}

esp_ip4_addr_t comm_wifi_get_ip(void) {
	return ip;
}

esp_ip4_addr_t comm_wifi_get_ip_client(void) {
	if (comm_local.socket > 0) {
		return comm_local.ip_client;
	} else {
		return comm_hub.ip_client;
	}
}

bool comm_wifi_is_client_connected(void) {
	return comm_local.socket >= 0 || comm_hub.socket >= 0;
}

bool comm_wifi_is_connecting(void) {
	return is_connecting;
}

bool comm_wifi_is_connected(void) {
	return is_connected;
}

void comm_wifi_disconnect(void) {
	if (comm_local.socket >= 0) {
		shutdown(comm_local.socket, 0);
		close(comm_local.socket);
		comm_local.socket = -1;
	}

	if (comm_hub.socket >= 0) {
		shutdown(comm_hub.socket, 0);
		close(comm_hub.socket);
		comm_hub.socket = -1;
	}
}

bool comm_wifi_change_network(const char *ssid, const char *password) {
	if (wifi_mode != WIFI_MODE_STATION) {
		return false;
	}

	if (password == NULL) {
		password = "";
	}

	size_t ssid_len = MIN(strlen(ssid), sizeof(wifi_config.sta.ssid) - 1);
	size_t password_len =
		MIN(strlen(password), sizeof(wifi_config.sta.password) - 1);

	memcpy(wifi_config.sta.ssid, ssid, ssid_len);
	wifi_config.sta.ssid[ssid_len] = '\0';

	memcpy(wifi_config.sta.password, password, password_len);
	wifi_config.sta.password[password_len] = '\0';

	wifi_config_changed = true;
	wifi_reconnect_disabled       = false;

	{
		esp_err_t result = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
		if (result == ESP_ERR_WIFI_PASSWORD) {
			STORED_LOGF(
				"incorrect wifi password, ssid: %p, password: %p", ssid,
				password
			);
			STORED_LOGF("ssid: '%s', password: '%s'", ssid, password);
			// TODO: Report incorrect password correctly. (Is incorrect password
			// even ever reported here?)
			return false;
		} else if (result != ESP_OK) {
			STORED_LOGF("esp_wifi_set_config failed, result: %d", result);

			return false;
		}
	}

	// I have no ****** idea if it would be ok to pass NULL here. I don't care
	// about the result, and only want to know if we're currently connected or
	// not. I'll just pass a temporary variable to be sure...
	wifi_ap_record_t temp;
	esp_err_t result = esp_wifi_sta_get_ap_info(&temp);
	bool connected   = result == ESP_OK;
	STORED_LOGF(
		"esp_wifi_sta_get_ap_info result: %d, is connected: %s", result,
		connected ? "true" : "false"
	);
	if (connected) {
		esp_err_t result = esp_wifi_disconnect();
		if (result != ESP_OK) {
			STORED_LOGF("esp_wifi_disconnect failed, result: %d", result);
			return false;
		}
	} else {
		esp_err_t result = esp_wifi_connect();
		if (result != ESP_OK) {
			STORED_LOGF("esp_wifi_connect failed, result: %d", result);
			return false;
		}
		is_connecting = true;
	}

	return true;
}

bool comm_wifi_disconnect_network() {
	if (wifi_mode != WIFI_MODE_STATION) {
		return false;
	}

	wifi_reconnect_disabled = true;
	is_connecting = false;

	comm_wifi_disconnect();
	esp_wifi_disconnect();

	return true;
}

bool comm_wifi_set_auto_reconnect(bool should_reconnect) {
	if (wifi_mode != WIFI_MODE_STATION) {
		return false;
	}
	
	wifi_auto_reconnect = should_reconnect;
	
	return true;
}

bool comm_wifi_get_auto_reconnect() {
	if (wifi_mode != WIFI_MODE_STATION) {
		return false;
	}
	
	return wifi_auto_reconnect;
}

void comm_wifi_set_event_listener(comm_wifi_event_cb_t handler) {
	event_listener = handler;
}

struct sockaddr_in create_sockaddr_in(ip_addr_t addr, uint16_t port) {
	struct sockaddr_in result = {0};
	// *Pretty* sure this works
	memcpy(&result.sin_addr, &addr, sizeof(ip_addr_t));
	result.sin_family = AF_INET;
	result.sin_port   = htons(port);
	// TODO: Is this necessary and correct if so?
	result.sin_len    = 4;

	return result;
}
