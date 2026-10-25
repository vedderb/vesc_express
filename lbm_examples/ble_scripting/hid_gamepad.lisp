; BLE HID button example
; 1. Enable the LispBM BLE client role, save and reboot.
; 2. Put your controller in pairing mode and run client_scan.lisp.
; 3. Copy its address and address type below, then run this script.
; Stop other scripts that read BLE client events.

(def controller-address [68 22 34 29 252 233])
(def controller-address-type 0)

; Change this function to react to button presses in your own application.
(defun on-button (report button pressed)
    (if pressed
        (print (list "Pressed button" button))
        (print (list "Released button" button))
    )
)

(defun start-example () {
    (print "Connecting to controller...")
    (connect-controller)

    (print "Finding buttons...")
    (find-buttons)
    (print (list "Buttons found:" (length button-fields)))
    (print "Ready. Press a button on your controller.")

    (loopwhile t {
        (read-controller)
        (sleep 0.001)
    })
})

; BLE and HID helpers
; The report map supplies button numbers, not labels such as A or B.
; Supports Variable Button fields; arrays are not decoded.

(def connection nil)
(def button-fields nil) ; report ID, bit offset, bit size, button number
(def input-reports nil) ; value handle, report ID, previous data
(def dropped 0)
(def drop-warning-time 0)

(defun read-unsigned (data start size) {
    (var value 0u64)

    (looprange i 0 size
        (setq value
            (bitwise-or value
                (shl (to-u64 (bufget-u8 data (+ start i))) (* i 8))
            )
        )
    )

    value
})

(defun uuid-is (uuid value)
    (and (= (buflen uuid) 2)
         (= (bufget-u8 uuid 0) (shr value 8))
         (= (bufget-u8 uuid 1) (bitwise-and value 255))
    )
)

; Read button locations from the report map. Other input fields and padding
; still advance the bit offset, but do not produce events.
(defun decode-button-map (data) {
    (setq button-fields nil)

    (var pos 0)
    (var page 0)
    (var size 0)
    (var count 0)
    (var id 0)
    (var globals nil)
    (var usages nil)
    (var usage-min nil)
    (var usage-max nil)
    (var offsets (bufcreate 1024))
    (bufclear offsets)

    (loopwhile (< pos (buflen data)) {
        (var prefix (bufget-u8 data pos))
        (setq pos (+ pos 1))
        (if (= prefix 0xfe) (exit-error 'unsupported-hid-long-item))

        (var len (bitwise-and prefix 3))
        (if (= len 3) (setq len 4))
        (if (> (+ pos len) (buflen data)) (exit-error 'truncated-report-map))
        (var value (read-unsigned data pos len))
        (setq pos (+ pos len))

        (var tag (bitwise-and prefix 0xfc))
        (cond
            ((= tag 0x04) (setq page value))
            ((= tag 0x74) (setq size value))
            ((= tag 0x94) (setq count value))

            ((= tag 0x84) {
                (if (or (= value 0) (> value 255)) (exit-error 'invalid-report-id))
                (setq id value)
            })

            ((= tag 0xa4)
                (setq globals (cons (list page size count id) globals))
            )
            ((= tag 0xb4) {
                (if (not globals) (exit-error 'invalid-global-pop))
                (var saved (car globals))
                (setq page (ix saved 0))
                (setq size (ix saved 1))
                (setq count (ix saved 2))
                (setq id (ix saved 3))
                (setq globals (cdr globals))
            })

            ((= tag 0x08)
                (setq usages (cons (if (= len 4) value (bitwise-or (shl page 16) value)) usages))
            )
            ((= tag 0x18)
                (setq usage-min (if (= len 4) value (bitwise-or (shl page 16) value)))
            )
            ((= tag 0x28)
                (setq usage-max (if (= len 4) value (bitwise-or (shl page 16) value)))
            )
            ((= tag 0xa8) (exit-error 'unsupported-usage-delimiter))

            ((= tag 0x80) {
                (var offset (bufget-u32 offsets (* id 4)))
                (var local (reverse usages))
                (var usage 0)

                (if (or (> count 256) (> (+ offset (* size count)) 4096))
                    (exit-error 'report-too-large))

                (if (and (= (bitwise-and value 3) 2) (> size 0) (<= size 16)) {
                    (looprange i 0 count {
                        (if local {
                            (setq usage (car local))
                            (setq local (cdr local))
                        } (if (and usage-min usage-max) {
                            (setq usage (+ usage-min i))
                            (if (> usage usage-max) (setq usage usage-max))
                        }))

                        (if (= (shr usage 16) 9)
                            (setq button-fields
                                (cons (list (to-i id) (to-i (+ offset (* i size))) (to-i size) (to-i (bitwise-and usage 65535))) button-fields)
                            )
                        )
                    })
                })

                (bufset-u32 offsets (* id 4) (+ offset (* size count))
                )
            })
        )

        ; Local usages apply only to the next Main item.
        (if (= (bitwise-and prefix 12) 0) {
            (setq usages nil)
            (setq usage-min nil)
            (setq usage-max nil)
        })
    })

    (setq button-fields (reverse button-fields))

})

(defun read-bits (data offset size) {
    (var shift (mod offset 8))
    (var bytes (// (+ shift size 7) 8))
    (var value
        (bitwise-and
            (shr (read-unsigned data (// offset 8) bytes) shift)
            (- (shl 1u64 size) 1)
        )
    )

    (to-i value)
})

; Report Reference selects the report ID; BLE payloads do not prepend it.
(defun handle-input-report (handle data)
    (map (lambda (report) {
        (if (= handle (ix report 0)) {
            (var previous (ix report 2))

            (map (lambda (field) {
                (if (= (ix field 0) (ix report 1)) {
                    (var offset (ix field 1))
                    (var size (ix field 2))

                    (if (> (+ offset size) (* (buflen data) 8))
                        (exit-error 'short-input-report))

                    (var pressed (!= (read-bits data offset size) 0))
                    (var old (if previous (!= (read-bits previous offset size) 0) nil))

                    (if (not (eq pressed old))
                        (on-button (ix field 0) (ix field 3) pressed))
                })
            }) button-fields)

            (setix report 2 data)
        })
    }) input-reports)
)

; Wait for setup operations while still consuming input notifications.
(defun check-request (result)
    (if (not (eq result t)) (exit-error (list 'request-failed result)))
)

(defun wait-for-event () {
    (var event nil)
    (var tries 0)

    (loopwhile (and (not event) (< tries 1500)) {
        (setq event (ble-client-event))
        (setq tries (+ tries 1))
        (if (not event) (sleep 0.01))
    })

    (if (not event) (exit-error 'event-timeout))
    (if (eq (car event) 'ble-disconnected) (exit-error event))
    (if (eq (car event) 'ble-passkey-action) (exit-error (cons 'pairing-needs-user-input event)))
    (if (and (eq (car event) 'ble-notification) (= (ix event 2) 0))
        (handle-input-report (ix event 1) (ix event 3))
    )
    event
})

(defun wait-for-result (kind) {
    (var event (wait-for-event))

    (loopwhile (not (eq (car event) kind)) (setq event (wait-for-event)))
    (if (!= (ix event 2) 0) (exit-error event))
    event
})

(defun collect-discovery () {
    (var records nil)
    (var event (wait-for-event))

    (loopwhile (not (eq (car event) 'ble-procedure-done)) {
        (setq records (cons event records))
        (setq event (wait-for-event))
    })
    (if (!= (ix event 2) 0) (exit-error event))
    (reverse records)
})

(defun read-value (handle long-read) {
    (check-request
        (if long-read
            (ble-client-read-long handle connection)
            (ble-client-read handle connection)
        )
    )
    (ix (wait-for-result 'ble-read-result) 3)
})

; Close a connection left by a previous run, then pair without PIN input.
(defun connect-controller () {
    ; Connections survive LispBM reloads. Close them before reconnecting.
    (if (ble-client-connect t) {
        (check-request (ble-client-connect nil))
        (var tries 0)

        (loopwhile (and (ble-client-connect t) (< tries 1500)) {
            (ble-client-event)
            (sleep 0.01)
            (setq tries (+ tries 1))
        })
        (if (ble-client-connect t) (exit-error 'disconnect-timeout))
    })

    (loopwhile (ble-client-event) nil)

    ; I/O 3: no input/display, with bonding.
    ; PIN-based pairing needs separate ble-client-passkey handling.
    (check-request (ble-client-connect controller-address controller-address-type 10000 3 t))
    (setq connection (ix (wait-for-result 'ble-connect-result) 1))
    (wait-for-result 'ble-security-result)

    (check-request (ble-client-exchange-mtu connection))
    (wait-for-result 'ble-mtu-result)
})

; Read the report map and subscribe to the controller's input reports.
(defun find-buttons () {
    ; Discover HID Service (0x1812) and its characteristics.
    (check-request (ble-client-discover-services connection))
    (var hid-service nil)
    (map (lambda (event)
        (if (and (eq (car event) 'ble-service) (uuid-is (ix event 3) 0x1812))
            (setq hid-service event))) (collect-discovery))
    (if (not hid-service) (exit-error 'no-ble-hid-service))

    (check-request (ble-client-discover-chars (ix hid-service 1) (ix hid-service 2) connection))
    (var characteristics nil)
    (map (lambda (event)
        (if (eq (car event) 'ble-characteristic)
            (setq characteristics (cons event characteristics)))) (collect-discovery))
    (setq characteristics (reverse characteristics))

    ; Use a long read for Report Map (0x2a4b).
    (map (lambda (chr)
        (if (uuid-is (ix chr 4) 0x2a4b)
            (decode-button-map (read-value (ix chr 1) t)))) characteristics)
    (if (not button-fields) (exit-error 'no-supported-buttons))

    ; Report Reference (0x2908) gives the report ID and type.
    ; Write the CCCD (0x2902) to enable input notifications.
    (var remaining characteristics)
    (loopwhile remaining {
        (var chr (car remaining))
        (setq remaining (cdr remaining))
        (if (uuid-is (ix chr 4) 0x2a4d) {
            ; Stop before the next characteristic declaration.
            (var end
                (if remaining (- (ix (car remaining) 2) 1) (ix hid-service 2)))
            (check-request (ble-client-discover-descriptors (ix chr 1) end connection))
            (var reference nil)
            (var cccd nil)
            (map (lambda (event)
                (if (eq (car event) 'ble-descriptor) {
                    (if (uuid-is (ix event 2) 0x2908) (setq reference (ix event 1)))
                    (if (uuid-is (ix event 2) 0x2902) (setq cccd (ix event 1)))
                })) (collect-discovery))
            (if reference {
                (var ref (read-value reference nil))
                (if (!= (buflen ref) 2) (exit-error 'invalid-report-reference))
                (if (and cccd (= (bufget-u8 ref 1) 1)) {
                    (setq input-reports (cons (list (ix chr 1) (bufget-u8 ref 0) nil) input-reports))
                    (check-request (ble-client-subscribe cccd 1 connection))
                    (wait-for-result 'ble-write-result)
                })
            })
        })
    })
    (if (not input-reports) (exit-error 'no-input-notifications))
    (setq dropped (ble-client-dropped))
    (setq drop-warning-time (systime))
})

; Process queued reports without printing unchanged button states.
(defun read-controller () {
    (var pending t)
    (var processed 0)

    (loopwhile (and pending (< processed 32)) {
        (var event (ble-client-event))
        (if event {
            (if (eq (car event) 'ble-disconnected) (exit-error event))
            (if (eq (car event) 'ble-notification) {
                (if (!= (ix event 2) 0) (exit-error event))
                (handle-input-report (ix event 1) (ix event 3))
            })
            (setq processed (+ processed 1))
        } (setq pending nil))
    })

    ; Limit overflow warnings to once per second.
    (var lost (ble-client-dropped))
    (if (and (!= dropped lost) (>= (secs-since drop-warning-time) 1.0)) {
        (print (list 'dropped-events (- lost dropped)))
        (setq dropped lost)
        (setq drop-warning-time (systime))
    })

})

(start-example)
