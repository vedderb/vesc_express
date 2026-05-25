; JFBMS Slave - CAN Protocol Implementation
; Broadcasts cell voltages and temperatures to master BMS via 11-bit CAN protocol
; Receives balance commands from master

; ============================================================================
; SLAVE CONFIGURATION - Read from VESC Tool configuration
; ============================================================================
(def slave-id (bms-get-slave-id))  ; Configured in VESC Tool -> JFBMS Slave -> Slave ID

; ============================================================================
; Cell Configuration (read from stored config)
; ============================================================================
(def cells-ic1 (bms-get-param 'cells_ic1))
(def cells-ic2 (bms-get-param 'cells_ic2))
(def total-cells (+ cells-ic1 cells-ic2))

; ============================================================================
; Balance Watchdog (counter-based, systime doesn't work in loops)
; ============================================================================
; Master must send balance command at least every 10 seconds
; If no command received, stop all balancing for safety
; Using counter: 100 iterations × 100ms = 10 seconds
(def bal-state (list 0))  ; state[0] = iterations since last balance command (0 = no cmd received)
; Flag: set to 1 when a balance command was received in process-can-messages
(def bal-rx-flag (list 0))
; Previous balance masks for change detection (only print when mask changes)
(def prev-ic1-mask (list 0))
(def prev-ic2-mask (list 0))
; Previous status flags printed to terminal
(def prev-status-flags (list -1))
; Settle tracking: count loops with zero balance bitmap
; After 20 loops (2s at 10Hz) voltages are considered settled for master balance decisions
(def settle-counter (list 20))  ; Start settled (no balancing at boot)

(defun trap-value (expr fallback) {
    (match (trap (eval expr))
        ((exit-ok (? value)) value)
        (_ fallback)
    )
})

; Balance visualization: show which cells are balancing per IC
; mask = 16-bit balance mask, ncells = number of configured cells
; Returns e.g. "_.X.X._" for cells 2,4 balancing out of 7
(defun bal-cells-str (mask ncells) {
    (var s "")
    (var active 0)
    (looprange i 0 ncells {
        (if (= (bitwise-and (shr mask i) 1) 1)
            { (setq s (str-merge s "X")) (setq active (+ active 1)) }
            (setq s (str-merge s "_"))
        )
    })
    (str-merge s " " (str-from-n active "%d") "/" (str-from-n ncells "%d"))
})

(defun tx-status-flags (bq1-ok bq2-ok) {
    (var flags 0)

    (if (not bq1-ok) (setq flags (+ flags 0x01)))
    ; CellsIC2 = 0 means there is no BQ2 fitted, not a BQ2 fault.
    (if (and (> cells-ic2 0) (not bq2-ok)) (setq flags (+ flags 0x02)))
    (if (>= (ix settle-counter 0) 20) (setq flags (+ flags 0x04)))

    flags
})

(defun print-tx-status (flags bq1-ok bq2-ok reason) {
    (print (str-merge "STATUS TX " reason
        " flags=0x" (str-from-n flags "%02X")
        " bq1=" (if bq1-ok "ok" "ERR")
        " bq2=" (if (> cells-ic2 0) (if bq2-ok "ok" "ERR") "none")
        " settled=" (if (> (bitwise-and flags 0x04) 0) "1" "0")
        " ic1=" (str-from-n cells-ic1 "%d")
        " ic2=" (str-from-n cells-ic2 "%d")))
})

(defun maybe-print-tx-status (bq1-ok bq2-ok force) {
    (var flags (tx-status-flags bq1-ok bq2-ok))

    (if (or force (not-eq flags (ix prev-status-flags 0))) {
        (print-tx-status flags bq1-ok bq2-ok (if force "repeat" "change"))
        (setix prev-status-flags 0 flags)
    })

    flags
})

(defun bool-u8 (v) (if v 1 0))

; ============================================================================
; Buzzer Beep Code Handler
; ============================================================================
; Beep codes received in byte 4 of balance command (DLC 5)
; Patterns defined here so they can be modified without reflashing firmware

(defun handle-beep (code)
    (cond
        ((= code 0x01) (buzzer-beep 2 100))    ; POWER_ON: 2 short beeps
        ((= code 0x02) (buzzer-beep 1 500))    ; POWER_OFF: 1 long beep
        ((= code 0x03) (buzzer-beep 3 100))    ; CHARGE_COMPLETE: 3 short beeps
        ((= code 0x04) (buzzer-beep 4 60))     ; SHUTDOWN: 4 fast beeps
        ((= code 0x10) (buzzer-beep 1 200))    ; ERR_OVER_TEMP: 1 beep
        ((= code 0x11) (buzzer-beep 2 200))    ; ERR_CELL_HIGH: 2 beeps
        ((= code 0x12) (buzzer-beep 3 200))    ; ERR_CELL_LOW: 3 beeps
        ((= code 0x13) (buzzer-beep 4 200))    ; ERR_OVERCURRENT: 4 beeps
        ((= code 0x14) (buzzer-beep 5 200))    ; ERR_BQ_COMM: 5 beeps
    )
)

; ============================================================================
; CAN RX Handler for Balance Commands from Master
; ============================================================================
; Balance command CAN ID: 0x500 | slave_id
; Data: 4 bytes balance bitmap (little-endian uint32) + optional byte 4 buzzer code
; Uses direct CAN buffer (bypasses broken event-can-sid system)

(defun process-can-messages () {
    (var expected-bal-id (+ 0x500 slave-id))
    (var rx-count 0)
    ; Process all available CAN messages
    (loopwhile (> (slave-can-available) 0) {
        (var msg (slave-can-read))
        (if msg {
            (setq rx-count (+ rx-count 1))
            (var can-id (car msg))
            (var data (cdr msg))
            (if (and (= can-id expected-bal-id)
                    (or (= (buflen data) 4) (= (buflen data) 5))) {
                ; Extract IC1 and IC2 masks separately (16-bit each, avoids 28-bit overflow)
                (var ic1-mask (+ (bufget-u8 data 0) (shl (bufget-u8 data 1) 8)))
                (var ic2-mask (+ (bufget-u8 data 2) (shl (bufget-u8 data 3) 8)))
                (var old-bal-active (or (> (ix prev-ic1-mask 0) 0)
                                         (> (ix prev-ic2-mask 0) 0)))
                (var new-bal-active (or (> ic1-mask 0) (> ic2-mask 0)))
                (var mask-changed (or (not-eq ic1-mask (ix prev-ic1-mask 0))
                                      (not-eq ic2-mask (ix prev-ic2-mask 0))))
                ; Apply to BQ immediately
                (var result (trap-value `(bms-set-bal-bitmap ,ic1-mask ,ic2-mask) false))
                (setix bal-rx-flag 0 1)
                (if result {
                    ; Reset watchdog counter (1 = command received, will increment each loop)
                    (setix bal-state 0 1)
                } {
                    (trap-value '(bms-stop-balancing) false)
                })

                ; Any transition into or out of active balancing makes voltages unsettled
                ; immediately. Only the zero-mask settle timer can set this true again.
                (if (and result (or old-bal-active new-bal-active)) {
                    (setix settle-counter 0 0)
                    (bms-set-settled-flag 0)
                })

                ; Only print when mask actually changes
                (if mask-changed {
                    (if result {
                        (setix prev-ic1-mask 0 ic1-mask)
                        (setix prev-ic2-mask 0 ic2-mask)
                        (print (str-merge "BAL: IC1=[" (bal-cells-str ic1-mask cells-ic1)
                            "] IC2=[" (bal-cells-str ic2-mask cells-ic2) "]"))
                    }
                        (print (str-merge "BAL FAIL: IC1=0x" (str-from-n ic1-mask "%04X")
                            " IC2=0x" (str-from-n ic2-mask "%04X"))))
                })

                ; Extract buzzer beep code from byte 4 if present (DLC >= 5)
                (if (>= (buflen data) 5) {
                    (var beep-code (bufget-u8 data 4))
                    (if (not (= beep-code 0)) (handle-beep beep-code))
                })
            })
        })
    })
    rx-count
})

; ============================================================================
; Balance Watchdog Check (counter-based)
; ============================================================================
; 100 iterations × 100ms = 10 seconds timeout
(def bal-watchdog-limit 100)

(defun check-bal-watchdog () {
    (var cnt (ix bal-state 0))
    ; Only check if we ever received a command (cnt > 0)
    (if (> cnt 0) {
        ; Increment counter
        (setix bal-state 0 (+ cnt 1))
        ; Check if timeout exceeded
        (if (> cnt bal-watchdog-limit) {
            (trap-value '(bms-stop-balancing) false)
            (setix bal-state 0 0)
            (print "Balance watchdog triggered - stopped balancing")
        })
    })
})

; ============================================================================
; Main Thread - Broadcast data every 100ms
; ============================================================================

(defun main-thd () {
    ; Track BQ communication status
    (var bq1-ok true)
    (var bq2-ok true)
    (var loop-count 0)

    (loopwhile t {
        (setix bal-rx-flag 0 0)

        ; Process incoming CAN messages (balance commands from master)
        (process-can-messages)

        ; Read cell voltages from both BQ chips
        (var cells (trap-value '(bms-get-vcells) nil))
        (if (eq cells nil) {
            (setq bq1-ok false)
            (setq cells '())
        } (setq bq1-ok true))

        ; Check CAN again after slow I2C reads so balance commands aren't delayed
        (process-can-messages)

        ; Read temperatures (BQ1-Int, BQ1-TS1, BQ2-Int, BQ2-TS1)
        (var temps (trap-value '(bms-get-temps) nil))
        (if (eq temps nil)
            (setq temps '(-273.0 -273.0 -273.0 -273.0)))

        ; Check CAN again after temperature I2C reads
        (process-can-messages)

        ; Check BQ2 status from temperature (invalid if < -200)
        (if (> cells-ic2 0) {
            (if (and (>= (length temps) 4) (> (ix temps 3) -200.0))
                (setq bq2-ok true)
                (setq bq2-ok false))
        } {
            ; Single-chip slaves report CellsIC2 = 0, not a BQ2 fault.
            (setq bq2-ok true)
        })

        ; Track settle state for master balance synchronization
        ; When balance FETs are off for >= 2s, voltages are settled and accurate
        (var bal-bmp-now (bms-get-bal-bitmap))
        (if (= bal-bmp-now 0) {
            ; Balance is off - increment settle counter toward threshold
            (if (< (ix settle-counter 0) 20)
                (setix settle-counter 0 (+ (ix settle-counter 0) 1)))
            (if (>= (ix settle-counter 0) 20)
                (bms-set-settled-flag 1))
        } {
            ; Balance is active - voltages are not settled
            (setix settle-counter 0 0)
            (bms-set-settled-flag 0)
        })

        (maybe-print-tx-status bq1-ok bq2-ok false)

        ; If a balance command was received, broadcast status immediately
        ; so master sees the updated balance mask without waiting for next cycle
        (if (= (ix bal-rx-flag 0) 1)
            (bms-broadcast-all slave-id cells temps (bool-u8 bq1-ok) (bool-u8 bq2-ok)))

        ; Debug: print every 10 loops (1 second)
        (setq loop-count (+ loop-count 1))
        (if (= (mod loop-count 10) 0) {
            ; Get balance bitmap and split into IC1/IC2 masks
            (var bal-bmp (bms-get-bal-bitmap))
            (var ic1-mask (bitwise-and bal-bmp 0xFFFF))
            (var ic2-mask (bitwise-and (shr bal-bmp 16) 0xFFFF))
            (var tx-flags (tx-status-flags bq1-ok bq2-ok))

            (print (str-merge "BQ1 bal: [" (bal-cells-str ic1-mask cells-ic1) "]"
                " 0x" (str-from-n ic1-mask "%04X")))
            (if (> cells-ic2 0)
                (print (str-merge "BQ2 bal: [" (bal-cells-str ic2-mask cells-ic2) "]"
                    " 0x" (str-from-n ic2-mask "%04X"))))
            (if (> (bitwise-and tx-flags 0x03) 0)
                (maybe-print-tx-status bq1-ok bq2-ok true))
        })

        ; Broadcast all data via CAN (cell msgs + 1 temp + 1 status)
        (bms-broadcast-all slave-id cells temps (bool-u8 bq1-ok) (bool-u8 bq2-ok))

        ; Update local VESC BMS values (for VESC Tool display when connected to slave)
        (slave-update-vesc-bms cells temps)

        ; Check balance watchdog
        (check-bal-watchdog)

        ; 100ms loop (10 Hz broadcast rate per protocol spec)
        (sleep 0.1)
    })
})

(defun main-supervisor () {
    (loopwhile t {
        (match (trap (main-thd))
            ((exit-ok _) (print "main-thd exited - restarting"))
            (_ (print "main-thd crashed - restarting"))
        )
        (sleep 0.1)
    })
})

; ============================================================================
; Startup
; ============================================================================

(print "JFBMS Slave starting...")
(print (str-merge "Slave ID: " (str-from-n slave-id "%d")))
(print (str-merge "Cells IC1: " (str-from-n cells-ic1 "%d") ", IC2: " (str-from-n cells-ic2 "%d")))

; Check if I2C device is present at 0x08
(print (str-merge "I2C detect 0x08: " (if (i2c-detect-addr 0x08) "OK" "FAIL")))

; Initialize BMS hardware
(def init-ok false)
(def bq1-init-ok false)
(def bq2-init-ok false)

(looprange i 0 10 {
    (if (bms-init cells-ic1 cells-ic2) {
        (setq init-ok true)
        (setq bq1-init-ok true)
        (setq bq2-init-ok (if (> cells-ic2 0) true false))
        (break)
    } {
        (print (str-merge "BMS init failed, attempt " (str-from-n (+ i 1) "%d") ", retrying..."))
        (sleep 1.0)
    })
})

; Set fault flags based on init status
(def fault-flags 0)
(if (not bq1-init-ok) (setq fault-flags (+ fault-flags 0x01)))
(if (and (> cells-ic2 0) (not bq2-init-ok)) (setq fault-flags (+ fault-flags 0x02)))
(bms-set-fault-flags fault-flags)

(if init-ok
    (print "BMS initialized successfully")
    (print "BMS init failed after 10 attempts - check I2C wiring and BQ76952 power"))

; Only continue if init was successful
(if init-ok {
    ; Start main broadcast loop (CAN RX is polled in main loop)
    (print "Starting CAN broadcast loop...")
    (spawn 200 main-supervisor)
} {
    ; Init failed - enter diagnostic loop, but still broadcast status
    (print "Entering diagnostic mode due to init failure")
    (var diag-count 0)
    (loopwhile t {
        (if (= (mod diag-count 10) 0)
            (print (str-merge "I2C detect 0x08: " (if (i2c-detect-addr 0x08) "OK" "FAIL"))))
        ; Broadcast empty data with fault flags
        (bms-broadcast-all slave-id '() '() 0 (if (> cells-ic2 0) 0 1))
        (setq diag-count (+ diag-count 1))
        (sleep 0.1)
    })
})
