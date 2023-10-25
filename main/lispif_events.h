#include "stdbool.h"
#include "lbm_types.h"

extern volatile bool event_can_sid_en;
extern volatile bool event_can_eid_en;
extern volatile bool event_data_rx_en;
extern volatile bool event_esp_now_rx_en;
extern volatile bool event_ble_rx_en;

extern volatile bool event_bms_bal_ovr_en;
extern volatile bool event_bms_chg_allow_en;
extern volatile bool event_bms_reset_cnt_en;
extern volatile bool event_bms_force_bal_en;
extern volatile bool event_bms_zero_ofs_en;

extern lbm_uint sym_event_can_sid;
extern lbm_uint sym_event_can_eid;
extern lbm_uint sym_event_data_rx;
extern lbm_uint sym_event_esp_now_rx;
extern lbm_uint sym_event_ble_rx;

extern lbm_uint sym_bms_chg_allow;
extern lbm_uint sym_bms_bal_ovr;
extern lbm_uint sym_bms_reset_cnt;
extern lbm_uint sym_bms_force_bal;
extern lbm_uint sym_bms_zero_ofs;

void lispif_events_load_symbols();