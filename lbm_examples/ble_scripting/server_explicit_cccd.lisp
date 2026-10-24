; Read, write and notify with an explicit CCCD.

(ble-start-app)
(map ble-remove-service (reverse (ble-get-services)))

(def handles (ble-add-service [0xbe 0xb5 0x48 0x3e] '(
    (
        (uuid . [0xbe 0xb5 0x48 0x4e])
        (prop . (prop-read prop-write prop-notify))
        (max-len . 100)
        (descr . (
            (
                (uuid . [0x29 0x02])
                (max-len . 2)
                (default-value . [0 0])
            )
        ))
    )
)))
(def service-handle (ix handles 0))
(def value-handle (ix handles 1))
(def cccd-handle (ix handles 2))

(print (ble-get-services))
(print (ble-get-attrs service-handle))

; String with a terminating zero byte.
(ble-attr-set-value value-handle "hello")
(print (ble-attr-get-value value-handle))

; Copy the string without its terminator.
(defun trim-null-byte (str) {
    (var len (str-len str))
    (var cpy (bufcreate len))
    (bufcpy cpy 0 str 0 len)
    cpy
})
(ble-attr-set-value value-handle (trim-null-byte "hello"))
(print (ble-attr-get-value value-handle))
(print (ble-attr-get-value cccd-handle))

; After subscribing, run in the REPL:
; (ble-attr-set-value value-handle [1 2 3])
