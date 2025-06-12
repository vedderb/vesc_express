#include "stdbool.h"
#include "lbm_types.h"
#include "symrepr.h"

// Idk if this is really the best way to handle this, but these variables need
// to be available in different source files.

volatile bool event_can_sid_en = false;
volatile bool event_can_eid_en = false;
volatile bool event_data_rx_en = false;
volatile bool event_esp_now_rx_en = false;
volatile bool event_ble_rx_en = false;
volatile bool event_wifi_disconnect_en = false;
volatile bool event_cmds_data_tx_en = false;

volatile bool event_bms_bal_ovr_en = false;
volatile bool event_bms_chg_allow_en = false;
volatile bool event_bms_reset_cnt_en = false;
volatile bool event_bms_force_bal_en = false;
volatile bool event_bms_zero_ofs_en = false;

lbm_uint sym_event_can_sid = 0;
lbm_uint sym_event_can_eid = 0;
lbm_uint sym_event_data_rx = 0;
lbm_uint sym_event_esp_now_rx = 0;
lbm_uint sym_event_ble_rx = 0;
lbm_uint sym_event_wifi_disconnect = 0;
lbm_uint sym_event_cmds_data_tx = 0;

lbm_uint sym_bms_chg_allow = 0;
lbm_uint sym_bms_bal_ovr = 0;
lbm_uint sym_bms_reset_cnt = 0;
lbm_uint sym_bms_force_bal = 0;
lbm_uint sym_bms_zero_ofs = 0;

void lispif_events_load_symbols() {
    lbm_add_symbol_const("event-can-sid", &sym_event_can_sid);
	lbm_add_symbol_const("event-can-eid", &sym_event_can_eid);
	lbm_add_symbol_const("event-data-rx", &sym_event_data_rx);
	lbm_add_symbol_const("event-esp-now-rx", &sym_event_esp_now_rx);
	lbm_add_symbol_const("event-ble-rx", &sym_event_ble_rx);
	lbm_add_symbol_const("event-wifi-disconnect", &sym_event_wifi_disconnect);
	lbm_add_symbol_const("event-cmds-data-tx", &sym_event_cmds_data_tx);

	lbm_add_symbol_const("event-bms-chg-allow", &sym_bms_chg_allow);
	lbm_add_symbol_const("event-bms-bal-ovr", &sym_bms_bal_ovr);
	lbm_add_symbol_const("event-bms-reset-cnt", &sym_bms_reset_cnt);
	lbm_add_symbol_const("event-bms-force-bal", &sym_bms_force_bal);
	lbm_add_symbol_const("event-bms-zero-ofs", &sym_bms_zero_ofs);
}