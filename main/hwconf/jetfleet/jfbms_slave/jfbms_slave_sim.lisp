; JFBMS Slave Simulator - Sends realistic fake cell/temp data via CAN
; No BQ76952 hardware required - for testing master CAN protocol

; ============================================================================
; Configuration - Read from VESC Tool package settings
; ============================================================================
(def slave-id (bms-get-slave-id))
(def cells-ic1 (bms-get-param 'cells_ic1))
(def cells-ic2 (bms-get-param 'cells_ic2))
(def total-cells (+ cells-ic1 cells-ic2))

; Initialize BMS with configured cell counts so C-level M_CELLS is set correctly
; This makes can_send_all_cells send the right number of messages and status sends correct cell_count
(bms-init cells-ic1 cells-ic2)

; ============================================================================
; Simulated Cell Base Voltages (mV, slightly different per cell)
; ============================================================================
; Range: ~3650mV to ~3750mV to mimic real pack imbalance
; Pool of 32 base values; only the first total-cells are used
(def cell-bases-pool (list
    3700.0 3749.0 3698.0 3747.0 3696.0 3745.0 3694.0 3743.0
    3692.0 3741.0 3690.0 3739.0 3688.0 3737.0 3686.0 3735.0
    3710.0 3659.0 3708.0 3657.0 3706.0 3655.0 3704.0 3653.0
    3702.0 3651.0 3720.0 3669.0 3718.0 3667.0 3716.0 3665.0
))
; Trim to configured cell count
(def cell-bases '())
(looprange i 0 total-cells
    (setq cell-bases (append cell-bases (list (ix cell-bases-pool i))))
)

; ============================================================================
; Simulated Temperature Base Values (deg C)
; ============================================================================
; T0: BQ1 die ~32, T1: Ext1 NTC ~26, T2: Ext2 NTC ~27, T3: BQ2 die ~33
(def temp-bases (list 32.0 26.0 27.0 33.0))

; ============================================================================
; Simple pseudo-random number generator (state in mutable list)
; ============================================================================
(def rng-state (list 12345))

(defun next-rng () {
    (var s (ix rng-state 0))
    (setix rng-state 0 (mod (+ (* s 1103) 12345) 65536))
    (ix rng-state 0)
})

; Flag: set to 1 when a balance command was received in process-can-messages
(def bal-rx-flag (list 0))

; ============================================================================
; CAN RX Handler for Balance Commands from Master
; ============================================================================
(defun process-can-messages () {
    (var expected-bal-id (+ 0x500 slave-id))
    (var rx-count 0)
    (setix bal-rx-flag 0 0)
    (loopwhile (> (slave-can-available) 0) {
        (var msg (slave-can-read))
        (if msg {
            (setq rx-count (+ rx-count 1))
            (var can-id (car msg))
            (var data (cdr msg))
            (if (= can-id expected-bal-id) {
                ; Extract IC1 and IC2 masks separately (16-bit each, avoids 28-bit overflow)
                (var ic1-mask (+ (bufget-u8 data 0) (shl (bufget-u8 data 1) 8)))
                (var ic2-mask (+ (bufget-u8 data 2) (shl (bufget-u8 data 3) 8)))
                ; Apply balance mask immediately
                (bms-set-bal-bitmap-demo ic1-mask ic2-mask)
                ; Read back actual mask to confirm
                (var actual (bms-get-bal-bitmap))
                (setix bal-rx-flag 0 1)
                (print (str-merge "SIM BAL RX: IC1=0x" (str-from-n ic1-mask "%04X")
                    " IC2=0x" (str-from-n ic2-mask "%04X")
                    " actual=0x" (str-from-n actual "%08X")
                    " t=" (str-from-n (systime) "%d")))

                ; Extract buzzer beep code from byte 4 if present (DLC >= 5)
                (if (>= (buflen data) 5) {
                    (var beep-code (bufget-u8 data 4))
                    (if (not (= beep-code 0)) {
                        (print (str-merge "SIM BEEP: 0x" (str-from-n beep-code "%02X")))
                        (buzzer-beep 4 60)
                    })
                })
            })
        })
    })
    rx-count
})

; ============================================================================
; Main Simulation Loop - Broadcast every 100ms
; ============================================================================

(defun main-thd () {
    (var loop-count 0)

    (loopwhile t {
        ; Process incoming CAN messages (balance + buzzer commands from master)
        (process-can-messages)

        ; Build cell voltage list with random drift
        (var cells '())
        (looprange i 0 total-cells {
            (var base (ix cell-bases i))
            ; Random drift: +/-10mV
            (var r (next-rng))
            (var drift (- (* (/ (to-float r) 32768.0) 10.0) 10.0))
            (var v (+ base drift))
            ; Clamp 3000-4200 mV
            (if (< v 3000.0) (setq v 3000.0))
            (if (> v 4200.0) (setq v 4200.0))
            ; Convert mV to V
            (setq cells (append cells (list (/ v 1000.0))))
        })

        ; Check CAN again after cell voltage generation (don't delay balance commands)
        (process-can-messages)

        ; Build temperature list with random drift +/-0.5 deg C
        (var temps '())
        (looprange i 0 4 {
            (var tb (ix temp-bases i))
            (var r (next-rng))
            (var drift (- (* (/ (to-float r) 32768.0) 0.5) 0.5))
            (setq temps (append temps (list (+ tb drift))))
        })

        ; Check CAN again after temp generation
        (process-can-messages)

        ; If a balance command was received, broadcast status immediately
        ; so master sees updated balance mask without waiting
        (if (= (ix bal-rx-flag 0) 1)
            (bms-broadcast-all slave-id cells temps true true))

        ; Debug print every 100 loops (10 seconds)
        (setq loop-count (+ loop-count 1))
        (if (= (mod loop-count 100) 0) {
            (print (str-merge "SIM loop " (str-from-n loop-count "%d")
                              " cells:" (str-from-n total-cells "%d")
                              " V0=" (str-from-n (ix cells 0) "%.3f")
                              "V V1=" (str-from-n (ix cells 1) "%.3f") "V"))
        })

        ; Broadcast via CAN
        (bms-broadcast-all slave-id cells temps true true)

        ; Update local VESC BMS display
        (slave-update-vesc-bms cells temps)

        ; 100ms interval (10 Hz, matches protocol spec)
        (sleep 0.1)
    })
})

; ============================================================================
; Startup
; ============================================================================

(print "=== JFBMS Slave SIMULATOR ===")
(print (str-merge "Slave ID: " (str-from-n slave-id "%d")))
(print (str-merge "Cells IC1: " (str-from-n cells-ic1 "%d") ", IC2: " (str-from-n cells-ic2 "%d")))
(print (str-merge "Simulated cells: " (str-from-n total-cells "%d")))
(print "Broadcasting fake data at 10 Hz (100ms)...")

(spawn 150 main-thd)
