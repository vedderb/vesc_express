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

#ifndef MAIN_COMM_WIFI_H_
#define MAIN_COMM_WIFI_H_

#include <stdint.h>
#include <stdbool.h>

#include "esp_netif.h"
#include "esp_event_base.h"
#include "lwip/sockets.h"
#include "lwip/ip_addr.h"

#include "datatypes.h"

#define CUSTOM_SOCKET_COUNT 10

/**
 * A event listener callback function used in the comm_wifi module.
 * event_base has many different possible values, signifying which "module" the
 * event came from. The definitions of these which are kind of spread out
 * though. The event_id integer then corresponds to the event_base's
 * corresponding event type enum.
 * Here are some example event bases:
 * - IP_EVENT   (from esp_netif_types.h, with event enum ip_event_t)
 * - WIFI_EVENT (from esp_wifi_types.h,  with event enum wifi_event_t)
*/
typedef void (*comm_wifi_event_cb_t)(esp_event_base_t event_base, int32_t event_id, void* event_data);

void comm_wifi_init(void);

WIFI_MODE comm_wifi_get_mode(void);
esp_ip4_addr_t comm_wifi_get_ip(void);
esp_ip4_addr_t comm_wifi_get_ip_client(void);
bool comm_wifi_is_client_connected(void);
bool comm_wifi_is_connecting(void);
bool comm_wifi_is_connected(void);

/**
 * Disconnect all sockets, including the vesc hub/local sockets and custom
 * sockets.
 * 
 * You should be carefull about where you call this, since this will shutdown
 * all sockets created in LispBM.
*/
void comm_wifi_disconnect(void);

/**
 * Connect to a new WIFI network.
 * 
 * This will close all existing TCP sockets!
 * 
 * The process has finshed when the IP_EVENT_STA_GOT_IP has fired in the event
 * listener.
 * 
 * @param ssid The SSID of the network to connect to. Should not be longer than
 * 31 characters (excluding the terminating null byte). Longer strings will be
 * trimmed.
 * @param password The password of the network to connect to. You may pass NULL
 * or an empty string if the network doesn't have a password (TODO: figure out
 * if this is actually the case). The same thing about const above is true here.
 * May not be longer than 63 characters (excluding the terminating null byte).
 * Longer strings will be trimmed.
 * @return A bool indicating if the password is correct. (TODO: Figure out
 * details.) If the wifi_mode isn't WIFI_MODE_STATION, false is returned.
*/
bool comm_wifi_change_network(const char *ssid, const char *password);

/**
 * Disconnect from the currently connected network, without automatically
 * reconnecting afterwards.
 * 
 * All open sockets will automatically be closed using the comm_wifi_disconnect
 * function.
 * 
 * Yes the naming is quite unfortunate, it should have been called
 * comm_wifi_disconnect, but it was already used...
 * 
 * @return True if the vesc is configured to be in station mode, false
 * otherwise.
*/
bool comm_wifi_disconnect_network();

// TODO: It kind of sucks that the wifi-connect
// LBM extension can return nil, while comm_wifi automatically retries in the
// background and successfully reconnects without notifying the user. But maybe
// that isn't that bad?
/**
 * Configure whether the comm_wifi module should automatically try to
 * reconnect wifi on disconnects.
 * 
 * @return False if wifi was not in station mode, or true otherwise.
*/
bool comm_wifi_set_auto_reconnect(bool should_reconnect);

/**
 * Get whetere the comm_wifi module will automatically try to reconnect wifi on
 * disconnects. Essentially just gets the value set by
 * comm_wifi_set_auto_reconnect.
 * 
 * @return The current setting, or false if wifi is not in station mode.
*/
bool comm_wifi_get_auto_reconnect();

/**
 * Create a new TCP socket managed by comm_wifi.
 * The returned socket is completely uninitialized as returned by the `socket()`
 * function, ready to be connected to a host with `connect()` for instance.
 * 
 * Be sure to close the created function using comm_wifi_close_socket!
 * If comm_wifi_socket_is_valid ever returns false for the created socket, it
 * has already been closed due to some internal process, and you don't need to
 * call comm_wifi_close_socket.
 * 
 * @return The successfully created socket, and -1 if there is already CUSTOM_SOCKET_COUNT created
 * sockets or the lwip function failed in creating a new socket. (I'm not sure
 * how the internal lwip_socket function works, so it might return negative
 * values other than -1 as well.)
*/
int comm_wifi_open_socket();
/**
 * Close a socket created by comm_wifi_open_socket.
 *
 * @param sock The socket to close, which was created using
 * `comm_wifi_open_socket`. It is not possible to close sockets created by other
 * meathods with this function.
 * @return True is returned if the socket existed, or false if it didn't exist
 * or was already closed.
*/
bool comm_wifi_close_socket(int sock);
/**
 * Check if a socket created by comm_wifi_open_socket is still valid.
 * 
 * @param sock The socket to check. May be negative.
 * @return If the socket was valid. False is returned if the socket has been
 * closed down or if socket was negative.
*/
bool comm_wifi_socket_is_valid(int sock);


/**
 * Register additional function that listens for ESP events.
 * 
 * Comm_wifi already has an ESP event handler that handles WIFI start, disconnect, and
 * connect events, so this function should not handles those events. See
 * the source code for more details about what the existing event handler does.
 * 
 * This function will be called after the internal event handler has already
 * processed the event and performed it's actions.
 * 
 * @param handler A function that will be called whenever an ESP event is
 * triggered. Any previously registered listeners will be overwritten. Pass NULL
 * to disable the existing event listener.
*/
void comm_wifi_set_event_listener(comm_wifi_event_cb_t handler);

void comm_wifi_send_packet_local(unsigned char *data, unsigned int len);
void comm_wifi_send_packet_hub(unsigned char *data, unsigned int len);
void comm_wifi_send_raw_local(unsigned char *buffer, unsigned int len);
void comm_wifi_send_raw_hub(unsigned char *buffer, unsigned int len);


// Utility functions

/**
 * Small convenience function to create a sockaddr_in struct from a ip_addr_t
 * and port. That specific address type is the same type as returned by
 * netconn_gethostbyname.
 * 
 * I created this because I'm sick of having to wrap my head around
 * archaic struct casting conventions...
 * 
 * @param addr The ipv4 address the return struct should refer to. Could for
 * example come from the netconn_gethostbyname function.
 * @param port The port to connect to. Is in host byte order.
*/
struct sockaddr_in create_sockaddr_in(ip_addr_t addr, uint16_t port);

#endif /* MAIN_COMM_WIFI_H_ */
