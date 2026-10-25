; Custom advertising using field lists.

(ble-set-name "test")
(ble-conf-adv-set-channels '(37 38 39))
(ble-conf-adv-set-interval 0x400 0x800)

(def adv-data '(
    (flags . [0x06])
    (name-complete . [0x74 0x65 0x73 0x74])
    (conn-interval-range . [0x06 0x00 0x30 0x00])
))
(def scan-rsp-data '(
    (flags . [0x06])
    (tx-power-level . [0x12])
    (conn-interval-range . [0x06 0x00 0x30 0x00])
))

(ble-conf-adv t adv-data scan-rsp-data)
(ble-start-app)
