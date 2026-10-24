; The same packets as advertising_fields.lisp, encoded as raw bytes.

(ble-set-name "test")
(ble-conf-adv-set-channels '(37 38 39))
(ble-conf-adv-set-interval 0x400 0x800)

(def adv-data [
    0x02 0x01 0x06                ; Flags
    0x05 0x09 0x74 0x65 0x73 0x74 ; Complete name: test
    0x05 0x12 0x06 0x00 0x30 0x00 ; Connection interval range
])
(def scan-rsp-data [
    0x02 0x01 0x06                ; Flags
    0x02 0x0a 0x12                ; Transmit power level
    0x05 0x12 0x06 0x00 0x30 0x00 ; Connection interval range
])

(ble-conf-adv t adv-data scan-rsp-data)
(ble-start-app)
