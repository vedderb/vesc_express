; Scan for 5 seconds and print BLE events.

(def result (ble-client-scan 5000))
(if (eq result t) {
    (var scanning t)
    (loopwhile scanning {
        (var event (ble-client-event))
        (if event {
            (print event)
            (setq scanning (not (eq (car event) 'ble-scan-done)))
        })
        (sleep 0.01)
    })
} (print (list 'scan-error result)))
