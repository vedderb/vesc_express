; Print writes received from a BLE client.

(defun event-handler ()
    (loopwhile t
        (recv
            ((event-ble-rx (? handle) (? data)) (print (list handle data)))
            (_ nil)
        )
    )
)

(ble-start-app)
(map ble-remove-service (reverse (ble-get-services)))
(event-register-handler (spawn event-handler))
(event-enable 'event-ble-rx)

(def handles (ble-add-service [0xbe 0xb5 0x48 0x3e] '(
    (
        (uuid . [0xbe 0xb5 0x48 0x4e])
        (prop . (prop-read prop-write))
        (max-len . 20)
        (default-value . [0])
    )
)))

(print (list 'write-to (ix handles 1)))
