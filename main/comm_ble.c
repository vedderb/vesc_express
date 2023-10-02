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

#include "comm_ble.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "console/console.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/ans/ble_svc_ans.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "commands.h"
#include "conf_general.h"
#include "main.h"
#include "packet.h"
#include "utils.h"
#include "crc.h"


#define GATTS_CHAR_VAL_LEN_MAX 255
#define PRINTF_MAX_LEN (400 - 1)

static size_t format_bytes_as_hex(
    const size_t dest_size, char dest[dest_size], const size_t n, const uint8_t bytes[n]
) {
    if (dest_size < 4) {
        int res = snprintf(dest, dest_size, "..");
        return (size_t)res;
    }
    
    size_t rest_n = 0;
    size_t first_n = n;
    size_t desired_len = n * 2;
    if (desired_len + 1 > dest_size) {
        size_t half_len = (dest_size - 1 - 3) / 2;
        rest_n = half_len / 2;
        first_n = rest_n;
    }
    
    int written = 0;
    for (size_t i = 0; i < first_n; i++) {
        int res = sprintf(dest + written, "%02x", bytes[i]);
        if (res == -1) {
            return written;
        }

        written += res;
    }
    
    if (rest_n != 0) {
        int res = sprintf(dest + written, "...");
        if (res == -1) {
            return written;
        }
        written += res;
        
        size_t offset = n - rest_n;
        
        for (size_t i = 0; i < rest_n; i++) {
            int res = sprintf(dest + written, "%02x", bytes[offset + i]);
            if (res == -1) {
                return written;
            }

            written += res;
        }
    }
    
    dest[written] = '\0';
    
    return (size_t)written;
}

// source: https://stackoverflow.com/a/5897216/15507414
#define VA_ARGS(...) , ##__VA_ARGS__
// #define log_printf(fmt, ...) commands_printf(fmt VA_ARGS(__VA_ARGS__))

static char error_message[128] = {0};
static send_func_t stored_send_func = NULL;

#define log_printf(fmt, ...)                                                   \
    {                                                                          \
        if (stored_send_func) {                                                \
            commands_start_send_func_overwrite(stored_send_func);              \
            commands_printf(fmt VA_ARGS(__VA_ARGS__));                         \
            commands_restore_send_func(stored_send_func);                      \
        } else {                                                               \
            commands_printf(fmt VA_ARGS(__VA_ARGS__));                         \
        }                                                                      \
    }

#define log_printf_only_stored(fmt, ...)                                       \
    {                                                                          \
        if (stored_send_func) {                                                \
            commands_start_send_func_overwrite(stored_send_func);              \
            commands_printf(fmt VA_ARGS(__VA_ARGS__));                         \
            commands_restore_send_func(stored_send_func);                      \
        }                                                                      \
    }

static void set_error_message(const char *message) {
    memcpy(
        error_message, message, MIN(strlen(message) + 1, sizeof(error_message))
    );
    error_message[sizeof(error_message) - 1] = '\0';
}

#define error_printf(fmt, ...)                                                 \
    snprintf(error_message, sizeof(error_message), fmt VA_ARGS(__VA_ARGS__))

static bool is_connected = false;
static uint16_t ble_current_mtu = 20;
static uint16_t notify_conn_handle = 0;

static uint8_t own_addr_type;

static PACKET_STATE_t *packet_state;

/**
 * Server service definition.
 */

static const ble_uuid128_t gatt_server_service_uuid = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
    0x01, 0x00, 0x40, 0x6E
);

static uint8_t chr_rx_str[GATTS_CHAR_VAL_LEN_MAX] = {0};
static uint16_t chr_rx_handle;
static const ble_uuid128_t chr_rx_uuid = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
    0x02, 0x00, 0x40, 0x6E
);

static uint8_t chr_tx_str[GATTS_CHAR_VAL_LEN_MAX] = {0};
static uint16_t chr_tx_handle;
static const ble_uuid128_t chr_tx_uuid = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
    0x03, 0x00, 0x40, 0x6E
);

static ble_uuid16_t dsc_client_cfg_uuid =
    BLE_UUID16_INIT(BLE_GATT_DSC_CLT_CFG_UUID16);

static int chr_rx_access(
    uint16_t conn_handle, uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt, void *arg
);
static int chr_tx_access(
    uint16_t conn_handle, uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt, void *arg
);
static int dsc_client_cfg_access(
    uint16_t conn_handle, uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt, void *arg
);

static const struct ble_gatt_svc_def gatt_server_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_server_service_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    // RX
                    // This characteristic can be subscribed to by writing 0x00
                    // and 0x01 to the CCCD
                    .uuid = &chr_rx_uuid.u,
                    .val_handle = &chr_rx_handle,
                    .access_cb = chr_rx_access,
                    .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                    // I have no idea what to put here...
                    .min_key_size = 0,
                    .descriptors =
                        (struct ble_gatt_dsc_def[]){
                            {
                                // Don't really understand why this is correct,
                                // but it works.
                                .uuid = &dsc_client_cfg_uuid.u,
                                .att_flags = BLE_ATT_F_READ,
                                .access_cb = dsc_client_cfg_access,
                            },
                            {0}, // No more descriptors
                        },
                },
                {
                    // TX
                    .uuid = &chr_tx_uuid.u,
                    .val_handle = &chr_tx_handle,
                    .access_cb = chr_tx_access,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                    .min_key_size = 0,
                    .descriptors =
                        (struct ble_gatt_dsc_def[]){
                            {
                                // Don't really understand why this is correct,
                                // but it works.
                                .uuid = &dsc_client_cfg_uuid.u,
                                .att_flags = BLE_ATT_F_READ,
                                .access_cb = dsc_client_cfg_access,
                            },
                            {0}, // No more descriptors
                        },
                },
                {0}, // No more characteristics
            },
    },
    {0}, // No more services.
};

static void process_data(const unsigned char *data, unsigned int len);

static int chr_rx_access(
    uint16_t conn_handle, uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt, void *arg
) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }
    
    notify_conn_handle = conn_handle;

    const struct os_mbuf *om = ctxt->om;

    uint16_t write_len;
    int rc =
        ble_hs_mbuf_to_flat(om, chr_rx_str, sizeof(chr_rx_str), &write_len);
    if (rc != 0) {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    process_data(chr_rx_str, write_len);

    ble_gatts_chr_updated(chr_rx_handle);

    return 0;
}

static int chr_tx_access(
    uint16_t conn_handle, uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt, void *arg
) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    // This should be pointless, since the only way data is transmitted over
    // this characteristic is through notifications...
    // chr_tx_str should always be empty...

    struct os_mbuf *om = ctxt->om;

    int rc = os_mbuf_append(om, &chr_tx_str, sizeof(chr_tx_str));
    if (rc != 0) {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return 0;
}

static int dsc_client_cfg_access(
    uint16_t conn_handle, uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt, void *arg
) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) {
        log_printf("tried to write to dsc_client_cfg, op: %u", ctxt->op);
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    // Umm don't really know what to do here...
    // let's assume that NimBLE has already taken care of what we need to do
    // here ;)

    return 0;
}

static void ble_advertise();

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that
 * forms. We use the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument, unused.
 *
 * @return                      0 if the application successfully handled
 * the event; nonzero on failure (this isn't really correct). The semantics
 *                              of the return code is specific to the
 *                              particular GAP event being signalled.
 */
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    log_printf("gap event: %u", event->type);
    
    // {
    //     char bytes[10];
    //     size_t written = format_bytes_as_hex(sizeof(bytes), bytes, 10, (uint8_t[]){0, 1, 2, 3, 4, 5, 6, 7, 8, 9});
    //     log_printf("written: %lu, bytes: '%s'", written, bytes);
    // }
    
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT: {
            // A new connection was established or a connection attempt failed.
            if (event->connect.status == 0) {
                log_printf(
                    "connection established, conn_handle: %d",
                    event->connect.conn_handle
                );
                is_connected = true;
            } else {
                log_printf(
                    "connection failed, status: %d", event->connect.status
                );

                ble_advertise();
            }

            return 0;
        }
        case BLE_GAP_EVENT_DISCONNECT: {
            is_connected = false;
            log_printf(
                "disconnected, conn_handle: %d, reason: %d",
                event->disconnect.conn.conn_handle, event->disconnect.reason
            );

            // Connection terminated, resume advertising.
            ble_advertise();
            return 0;
        }
        case BLE_GAP_EVENT_CONN_UPDATE: {
            // The central has updated the connection parameters.
            return 0;
        }
        case BLE_GAP_EVENT_CONN_UPDATE_REQ: {
            // The central has requested updated connection parameters.
            return 0;
        }
        case BLE_GAP_EVENT_ADV_COMPLETE: {
            ble_advertise();
            return 0;
        }
        case BLE_GAP_EVENT_ENC_CHANGE: {
            // Encryption has been enabled or disabled for this connection.
            log_printf("TODO");
            return 0;
        }
        case BLE_GAP_EVENT_NOTIFY_TX: {
            // The central device has been notified.
            return 0;
        }
        case BLE_GAP_EVENT_SUBSCRIBE: {
            // The central device has subscribes to a characteristic.
            log_printf(
                "conn_handle=%d, subscription for attribute %d: notify: %s "
                "(prev: %s), indicate: %s (prev: %s)",
                event->subscribe.conn_handle, event->subscribe.attr_handle,
                event->subscribe.cur_notify ? "true" : "false",
                event->subscribe.prev_notify ? "true" : "false",
                event->subscribe.cur_indicate ? "true" : "false",
                event->subscribe.prev_indicate ? "true" : "false"
            );
            return 0;
        }
        case BLE_GAP_EVENT_MTU: {
            // The MTU has been updated.

            ble_current_mtu = event->mtu.value;
            log_printf("update mtu: %u", ble_current_mtu);
            return 0;
        }
        case BLE_GAP_EVENT_REPEAT_PAIRING: {
            // TODO: figure out what is happening here.
            /** We already have a bond with the peer, but it is attempting to
             * establish a new secure link.  This app sacrifices security for
             * convenience: just throw away the old bond and accept the new
             * link.
             */

            // Delete the old bond.
            struct ble_gap_conn_desc desc;
            int rc =
                ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
            if (rc != 0) {
                return 0;
            };
            ble_store_util_delete_peer(&desc.peer_id_addr);

            /** Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host
             * should continue with the pairing operation.
             */
            return BLE_GAP_REPEAT_PAIRING_RETRY;
        }
        case BLE_GAP_EVENT_PASSKEY_ACTION: {
            log_printf("TODO");

            return 0;
        }
        default: {
            return 0;
        }
    }
}

/**
 * Enables advertising.
 */
static void ble_advertise() {
    int rc;

    struct ble_hs_adv_fields fields;

    /**
     *  Set the data included in our base advertisements:
     *     - Flags (indicates advertisement type and other general info).
     *     - Device name.
     *     - 128-bit service UUIDs.
     */
    memset(&fields, 0, sizeof(fields));

    /** Advertise two flags:
     *     - Discoverability in forthcoming advertisement (general)
     *     - BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    const char *name = ble_svc_gap_device_name();

    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids128 = (ble_uuid128_t[]){gatt_server_service_uuid};
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        error_printf("adv set fields fail, rc=%d", rc);
        return;
    }

    /**
     *  Set the data included in advertisement scan responses:
     *     - Flags (indicates advertisement type and other general info).
     *     - Advertising tx power.
     *     - Device name.
     *     - Slave connection interval range.
     */
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /** Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.slave_itvl_range = (uint8_t[]){0x06, 0x00, 0x30, 0x00};

    rc = ble_gap_adv_rsp_set_fields(&fields);
    if (rc != 0) {
        error_printf("adv scan rsp set fields fail, rc=%d", rc);
        return;
    }

    // Begin advertising
    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(
        own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL
    );
    if (rc != 0) {
        error_printf("adv start fail, rc=%d", rc);
        return;
    }
}

/**
 * Is called when the host and controller are reset due to a fatal error.
 */
static void ble_on_reset(int reason) {
    log_printf("reseting ble state, reason: %d", reason);
}

/**
 * Is called when the host and controller are synced. This occurs on startup and
 * after a reset.
 */
static void ble_on_sync(void) {
    log_printf("host and controller synced");

    int rc;

    // Make sure we have proper identity address set (public preferred)
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        error_printf("ble_hs_util_ensure_addr fail, rc=%d", rc);
    }

    // Figure out address to use while advertising (no privacy for now)
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        error_printf("couldn't determine address type, rc=%d", rc);
        return;
    }

    ble_advertise();
}

void ble_host_task(void *param) {
    // This function will return only when nimble_port_stop() is executed.
    nimble_port_run();

    nimble_port_freertos_deinit();
}

static int gatt_server_init(void) {
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_ans_init();

    rc = ble_gatts_count_cfg(gatt_server_services);
    if (rc != 0) {
        error_printf("gatts count cfg fail, rc=%d", rc);
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_server_services);
    if (rc != 0) {
        error_printf("gatts add services fail, rc=%d", rc);
        return rc;
    }
    
    return 0;
}

static void process_packet(unsigned char *data, unsigned int len) {
    commands_process_packet(data, len, comm_ble_send_packet);
}

static void process_data(const unsigned char *data, unsigned int len) {
    char bytes[MIN(len * 2, PRINTF_MAX_LEN - 30) + 1];
    format_bytes_as_hex(sizeof(bytes), bytes, len, (uint8_t *)data);
    log_printf("recv %d, 0x%s.", len, bytes);

    for (int i = 0; i < len; ++i) {
        packet_process_byte(data[i], packet_state);
    }
}

static void send_packet_raw(unsigned char *buffer, unsigned int len) {
    if (!is_connected) {
        return;
    }

    {
        char bytes[MIN(len * 2, PRINTF_MAX_LEN - 30) + 1];
        format_bytes_as_hex(sizeof(bytes), bytes, len, (uint8_t *)buffer);
        log_printf_only_stored("sent %d, 0x%s.", len, bytes);
    }

    uint16_t bytes_sent = 0;

    while (bytes_sent < len) {
        uint16_t length = 0;
        if (len - bytes_sent > ble_current_mtu - 3) {
            length = ble_current_mtu - 3;
        } else {
            length = len - bytes_sent;
        }

        if (length == 0 || length > ble_current_mtu - 3) {
            error_printf("invalid packet unit len: %u", length);
            log_printf_only_stored(
                "invalid packet unit length, bytes_sent: %u, length: %u, "
                "ble_current_mtu - 3: %u, len: %u",
                bytes_sent, length, ble_current_mtu - 3, len
            );
            break;
        }

        struct os_mbuf *data =
            ble_hs_mbuf_from_flat(buffer + bytes_sent, length);
        
        {
            char bytes[MIN(length * 2, PRINTF_MAX_LEN - 30) + 1];
            format_bytes_as_hex(sizeof(bytes), bytes, length, (uint8_t *)buffer + bytes_sent);
            log_printf_only_stored("sent unit %d, 0x%s.", length, bytes);
        }
        int rc = ble_gatts_notify_custom(notify_conn_handle, chr_tx_handle, data);
        if (rc != 0) {
            log_printf_only_stored("notify failed, rc=%d", rc);
            break;
        }
		vTaskDelay(100 / portTICK_PERIOD_MS);

        bytes_sent += length;
    }
}

void ble_store_config_init(void);

void comm_ble_init(void) {
    packet_state = calloc(1, sizeof(PACKET_STATE_t));
    packet_init(send_packet_raw, process_packet, packet_state);

    int rc;

    rc = nimble_port_init();
    if (rc != ESP_OK) {
        error_printf("nimble port init fail, rc=%d", rc);
        return;
    }

    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    rc = gatt_server_init();
    if (rc != 0) {
        return;
    }

    rc = ble_svc_gap_device_name_set((char *)backup.config.ble_name);
    if (rc != 0) {
        error_printf("device name set fail, rc=%d", rc);
        return;
    }

    // Honestly don't understand what this does entirely...
    ble_store_config_init();

    nimble_port_freertos_init(ble_host_task);
}

bool comm_ble_is_connected() { return is_connected; }

int comm_ble_mtu_now(void) { return ble_current_mtu; }

const char *comm_ble_get_error_message() { return error_message; }
const char *comm_ble_get_message() { return ""; }
void comm_ble_print_chr() {
    commands_printf("chr_rx_handle: %u", chr_rx_handle);
    commands_printf("chr_tx_handle: %u", chr_tx_handle);
}

void comm_ble_send_packet(unsigned char *data, unsigned int len) {
    packet_send_packet(data, len, packet_state);
}

// TODO: remove this
void comm_ble_store_curr_send_func() {
    stored_send_func = commands_get_send_func();
}
send_func_t comm_ble_get_stored_send_func() { return stored_send_func; }