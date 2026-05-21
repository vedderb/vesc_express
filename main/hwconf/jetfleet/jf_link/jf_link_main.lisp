; JF Link - JFBMS master controller
; Lisp owns the UI-driven behavior; C only buffers CAN and exposes slave data.

(print "=== JF Link Master ===")

; Reset slave data
(master-reset-slaves)

; Previous active state for each slave (1-8)
(def prev-active (list 0 0 0 0 0 0 0 0))

; Balancing state
; state[0] = 1 if currently balancing, 0 if not
(def bal-state (list 0))
(def manual-bal-state (list 0))
(def bal-request false)

; Cached slave masks, updated by balance thread and transmitted by main loop keepalive
(def slave-bal-mask-ic1 (list 0 0 0 0 0 0 0 0))
(def slave-bal-mask-ic2 (list 0 0 0 0 0 0 0 0))

; One-shot request for immediate keepalive TX
(def bal-keepalive-kick (list 0))

; Runtime defaults. Keep these here instead of reading bms-get-param so the app
; also works on JF Link firmware that does not expose JF Link config symbols yet.
(def cfg-num-slaves-value 1)
(def cfg-slave-timeout-s-value 1.0)
(def cfg-max-bal-ch-value 8)
(def cfg-bal-start-value 0.005)
(def cfg-bal-end-value 0.003)
(def cfg-bal-min-value 3.5)
(def cfg-param-state (list -1)) ; -1 unknown, 0 unavailable, 1 available

(defun cfg-num-slaves () {
    (var n cfg-num-slaves-value)
    (looprange i 1 9 {
        (if (master-slave-active? i) (setq n i))
    })
    (if (< n 1) 1 (if (> n 8) 8 n))
})

(defun cfg-slave-timeout-s () cfg-slave-timeout-s-value)
(defun cfg-max-bal-ch () cfg-max-bal-ch-value)
(defun cfg-bal-start () cfg-bal-start-value)
(defun cfg-bal-end () cfg-bal-end-value)
(defun cfg-bal-min () cfg-bal-min-value)

(defun cfg-read-param (name fallback) {
    (if (= (ix cfg-param-state 0) 0) fallback {
        (match (trap (bms-get-param name))
            ((exit-ok (? value)) {
                (setix cfg-param-state 0 1)
                value
            })
            (_ {
                (if (= (ix cfg-param-state 0) -1)
                    (print "CFG: bms-get-param unavailable, using Lisp defaults"))
                (setix cfg-param-state 0 0)
                fallback
            })
        )
    })
})

(defun cfg-refresh () {
    (setq cfg-num-slaves-value (cfg-read-param 'num_slaves cfg-num-slaves-value))
    (setq cfg-slave-timeout-s-value (cfg-read-param 'slave_timeout_s cfg-slave-timeout-s-value))
    (setq cfg-max-bal-ch-value (cfg-read-param 'max_bal_ch cfg-max-bal-ch-value))
    (setq cfg-bal-start-value (cfg-read-param 'vc_balance_start cfg-bal-start-value))
    (setq cfg-bal-end-value (cfg-read-param 'vc_balance_end cfg-bal-end-value))
    (setq cfg-bal-min-value (cfg-read-param 'vc_balance_min cfg-bal-min-value))

    (if (= (ix cfg-param-state 0) 1)
        (print (str-merge "CFG: slaves=" (str-from-n cfg-num-slaves-value "%d")
            " start=" (str-from-n cfg-bal-start-value "%.3f")
            " end=" (str-from-n cfg-bal-end-value "%.3f")
            " min=" (str-from-n cfg-bal-min-value "%.3f")
            " max-ch=" (str-from-n cfg-max-bal-ch-value "%d"))))
})

; Pick non-adjacent cells from one BQ76952 group. Cells are split into even and
; odd local indexes, then the stronger group is selected.
(defun balance-ic-group (voltages c-min threshold max-ch) {
    (var n (length voltages))
    (if (= n 0) 0 {
        (var even-grp '())
        (var odd-grp '())

        (looprange i 0 n {
            (var v (ix voltages i))
            (if (> (- v c-min) threshold) {
                (if (= (mod i 2) 0)
                    (setq even-grp (cons (cons i v) even-grp))
                    (setq odd-grp (cons (cons i v) odd-grp)))
            })
        })

        (var even-sorted (sort (fn (a b) (> (cdr a) (cdr b))) even-grp))
        (var odd-sorted (sort (fn (a b) (> (cdr a) (cdr b))) odd-grp))
        (var use-even true)

        (if (> (length odd-sorted) (length even-sorted))
            (setq use-even false))

        (if (= (length odd-sorted) (length even-sorted)) {
            (var even-sum 0.0)
            (loopforeach p even-sorted
                (setq even-sum (+ even-sum (- (cdr p) c-min))))

            (var odd-sum 0.0)
            (loopforeach p odd-sorted
                (setq odd-sum (+ odd-sum (- (cdr p) c-min))))

            (if (> odd-sum even-sum) (setq use-even false))
        })

        (var grp (if use-even even-sorted odd-sorted))
        (var mask 0)
        (var cnt 0)

        (loopforeach c grp {
            (if (>= cnt max-ch) (break))
            (setq mask (+ mask (shl 1 (car c))))
            (setq cnt (+ cnt 1))
        })

        mask
    })
})

(defun mask-to-bin (mask n) {
    (var s "")
    (looprange i 0 n {
        (setq s (str-merge s (if (> (bitwise-and mask (shl 1 i)) 0) "1" "0")))
    })
    s
})

(defun mask-apply-bit (mask bit enable) {
    (if (> enable 0) {
        (if (= (bitwise-and mask bit) 0) (+ mask bit) mask)
    } {
        (if (> (bitwise-and mask bit) 0) (- mask bit) mask)
    })
})

(defun any-cached-balancing () {
    (var any false)
    (looprange i 0 8 {
        (if (or (> (ix slave-bal-mask-ic1 i) 0) (> (ix slave-bal-mask-ic2 i) 0))
            (setq any true))
    })
    any
})

(defun clear-cached-balancing () {
    (looprange i 0 8 {
        (setix slave-bal-mask-ic1 i 0)
        (setix slave-bal-mask-ic2 i 0)
    })
})

(defun send-cached-balance-masks (beep-code) {
    (var sid 1)
    (var max-sid (cfg-num-slaves))

    (loopwhile (<= sid max-sid) {
        (if (master-slave-active? sid) {
            (master-send-balance
                sid
                (ix slave-bal-mask-ic1 (- sid 1))
                (ix slave-bal-mask-ic2 (- sid 1))
                beep-code)
        })
        (setq sid (+ sid 1))
    })
})

(defun stop-all-balancing () {
    (clear-cached-balancing)
    (send-cached-balance-masks 0)
    (setix bal-keepalive-kick 0 0)
    (setix bal-state 0 0)
    (setix manual-bal-state 0 0)
})

(defun set-manual-balance-cell (cell enable) {
    (master-can-read-all)

    (var sid 1)
    (var base 0)
    (var done false)
    (var max-sid (cfg-num-slaves))

    (loopwhile (and (<= sid max-sid) (not done)) {
        (if (master-slave-active? sid) {
            (var cells (master-get-slave-cells sid))
            (var ic1-cnt (master-get-cells-ic1 sid))
            (var ic2-cnt (master-get-cells-ic2 sid))
            (var cnt (+ ic1-cnt ic2-cnt))

            (if (and cells (> cnt 0) (= (length cells) cnt)) {
                (if (and (>= cell base) (< cell (+ base cnt))) {
                    (var local (- cell base))
                    (var bit 0)

                    (if (< local ic1-cnt) {
                        (setq bit (shl 1 local))
                        (setix slave-bal-mask-ic1 (- sid 1)
                            (mask-apply-bit (ix slave-bal-mask-ic1 (- sid 1)) bit enable))
                    } {
                        (setq bit (shl 1 (- local ic1-cnt)))
                        (setix slave-bal-mask-ic2 (- sid 1)
                            (mask-apply-bit (ix slave-bal-mask-ic2 (- sid 1)) bit enable))
                    })

                    (master-send-balance
                        sid
                        (ix slave-bal-mask-ic1 (- sid 1))
                        (ix slave-bal-mask-ic2 (- sid 1))
                        0)

                    (setix manual-bal-state 0 (if (any-cached-balancing) 1 0))
                    (setix bal-keepalive-kick 0 1)
                    (print (str-merge "BAL OVR: cell " (str-from-n cell "%d")
                        (if (> enable 0) " on" " off")
                        " -> S" (str-from-n sid "%d")
                        " IC1:" (mask-to-bin (ix slave-bal-mask-ic1 (- sid 1)) ic1-cnt)
                        " IC2:" (mask-to-bin (ix slave-bal-mask-ic2 (- sid 1)) ic2-cnt)))
                    (setq done true)
                })

                (setq base (+ base cnt))
            })
        })
        (setq sid (+ sid 1))
    })

    (if (not done)
        (print (str-merge "BAL OVR: cell " (str-from-n cell "%d") " not found")))

    done
})

(defun event-handler ()
    (loopwhile t
        (recv
            ((event-bms-bal-ovr (? cell) (? enable)) {
                (setix bal-state 0 0)
                (setq bal-request false)
                (set-manual-balance-cell cell (if (> enable 0) 1 0))
            })
            ((event-bms-force-bal (? v)) {
                (if (= v 1) (cfg-refresh))
                (setq bal-request (= v 1))
                (if bal-request {
                    (setix manual-bal-state 0 0)
                    (setix bal-keepalive-kick 0 1)
                    (print "BAL CMD: start")
                } {
                    (print "BAL CMD: stop")
                    (stop-all-balancing)
                })
            })
            (_ nil)
)))

(defun balance-thd () (loopwhile t {
    (var max-ch (cfg-max-bal-ch))
    (var bal-start (cfg-bal-start))
    (var bal-end (cfg-bal-end))
    (var bal-min (cfg-bal-min))
    (var is-bal (= (ix bal-state 0) 1))
    (var threshold (if is-bal bal-end bal-start))

    (if (not bal-request) {
        (if (and (= (ix bal-state 0) 1) (= (ix manual-bal-state 0) 0)) {
            (print "BAL: stopped by command")
            (stop-all-balancing)
        })
        (sleep 0.2)
    } {
        ; Phase 1: stop balancing and wait for settled voltages.
        (clear-cached-balancing)
        (send-cached-balance-masks 0)

        (var settle-wait 0)
        (var max-settle-wait 50) ; 50 x 100ms = 5s timeout
        (var settled-ready false)

        (loopwhile (and bal-request (< settle-wait max-settle-wait)) {
            (sleep 0.1)
            (master-can-read-all)
            (setq settle-wait (+ settle-wait 1))

            (if (>= settle-wait 20) {
                (var all-settled true)
                (var active-count 0)
                (var chk-sid 1)
                (var max-sid (cfg-num-slaves))

                (loopwhile (<= chk-sid max-sid) {
                    (if (master-slave-active? chk-sid) {
                        (setq active-count (+ active-count 1))
                        (if (not (master-get-slave-settled? chk-sid))
                            (setq all-settled false))
                    })
                    (setq chk-sid (+ chk-sid 1))
                })

                (if (and (> active-count 0) all-settled) {
                    (setq settled-ready true)
                    (break)
                })
            })
        })

        (master-can-read-all)

        ; Phase 2: compute balance masks from settled slave cell voltages.
        (if bal-request {
            (if (not settled-ready) {
                (print "BAL: stopped (slaves did not report settled status)")
                (stop-all-balancing)
                (setq bal-request false)
            })
        })

        (if bal-request {
            (var global-min 9.0)
            (var global-max 0.0)
            (var any-cells false)
            (var valid-slaves 0)
            (var total-cells 0)
            (var slave-data '())
            (var sid 1)
            (var max-sid (cfg-num-slaves))

            (loopwhile (<= sid max-sid) {
                (if (master-slave-active? sid) {
                    (var cells (master-get-slave-cells sid))
                    (var s-ic1 (master-get-cells-ic1 sid))
                    (var s-ic2 (master-get-cells-ic2 sid))
                    (var cnt (+ s-ic1 s-ic2))

                    (if (and cells (> cnt 0) (= (length cells) cnt)) {
                        (setq any-cells true)
                        (setq valid-slaves (+ valid-slaves 1))
                        (setq total-cells (+ total-cells cnt))
                        (setq slave-data (cons (list sid cells s-ic1 s-ic2) slave-data))
                        (loopforeach v cells {
                            (if (< v global-min) (setq global-min v))
                            (if (> v global-max) (setq global-max v))
                        })
                    })
                })
                (setq sid (+ sid 1))
            })

            (if any-cells
                (print (str-merge "BAL: scan slaves=" (str-from-n valid-slaves "%d")
                    " cells=" (str-from-n total-cells "%d")
                    " min=" (str-from-n global-min "%.3f")
                    " max=" (str-from-n global-max "%.3f")
                    " delta=" (str-from-n (- global-max global-min) "%.3f")
                    " thr=" (str-from-n threshold "%.3f"))))

            (if (and any-cells (> global-min bal-min)) {
                (var any-bal false)

                (loopforeach sd slave-data {
                    (var s-id (ix sd 0))
                    (var cells (ix sd 1))
                    (var ic1-cnt (ix sd 2))
                    (var ic2-cnt (ix sd 3))

                    (var ic1-volts (map (fn (i) (ix cells i)) (range ic1-cnt)))
                    (var ic2-volts (if (> ic2-cnt 0)
                        (map (fn (i) (ix cells (+ ic1-cnt i))) (range ic2-cnt))
                        '()))

                    (var ic1-mask (balance-ic-group ic1-volts global-min threshold max-ch))
                    (var ic2-mask (if (> ic2-cnt 0)
                        (balance-ic-group ic2-volts global-min threshold max-ch)
                        0))

                    (if (or (> ic1-mask 0) (> ic2-mask 0)) {
                        (setq any-bal true)
                        (print (str-merge "BAL S" (str-from-n s-id "%d")
                            " IC1:" (mask-to-bin ic1-mask ic1-cnt)
                            " IC2:" (mask-to-bin ic2-mask ic2-cnt)
                            " min=" (str-from-n global-min "%.3f")))
                    })

                    (setix slave-bal-mask-ic1 (- s-id 1) ic1-mask)
                    (setix slave-bal-mask-ic2 (- s-id 1) ic2-mask)
                })

                (if any-bal {
                    (setix bal-state 0 1)
                    (setix bal-keepalive-kick 0 1)

                    ; Phase 3: hold for about 30s. Main loop sends keepalive at 1Hz.
                    (var hold-cnt 0)
                    (loopwhile (and bal-request (< hold-cnt 30)) {
                        (sleep 1.0)
                        (setq hold-cnt (+ hold-cnt 1))
                    })
                } {
                    (setix bal-state 0 0)
                    (setq bal-request false)
                    (print (str-merge "BAL: target reached (delta "
                        (str-from-n (- global-max global-min) "%.3f")
                        " <= " (str-from-n threshold "%.3f") ")"))
                })
            } {
                (stop-all-balancing)
                (setq bal-request false)
                (if any-cells
                    (print (str-merge "BAL: stopped (min " (str-from-n global-min "%.3f")
                        " <= " (str-from-n bal-min "%.3f") ")"))
                    (print "BAL: stopped (no cells available)"))
            })
        })
    })

    (if (and (not bal-request) (= (ix bal-state 0) 1)) (stop-all-balancing))
}))

; Register BMS command events used by VESC Tool
(event-register-handler (spawn event-handler))
(event-enable 'event-bms-bal-ovr)
(event-enable 'event-bms-force-bal)

(cfg-refresh)

(print "Spawning balance thread...")
(spawn 200 balance-thd)
(print "Balance thread spawned")

(print "Entering JF Link main loop...")
(def loop-cnt 0)

(loopwhile t {
    ; Drain CAN every 50ms.
    (master-can-read-all)

    ; 10Hz display/status work.
    (if (= (mod loop-cnt 2) 0) {
        (if (and (or (and bal-request (= (ix bal-state 0) 1))
                     (= (ix manual-bal-state 0) 1))
                 (or (= (mod loop-cnt 20) 0) (= (ix bal-keepalive-kick 0) 1))) {
            (send-cached-balance-masks 0)
            (setix bal-keepalive-kick 0 0)
        })

        (master-update-vesc-bms)
        (master-send-bms-can)
        (master-check-timeouts (cfg-slave-timeout-s))

        (var id 1)
        (var max-id (cfg-num-slaves))

        (loopwhile (<= id max-id) {
            (var active (if (master-slave-active? id) 1 0))
            (var prev (ix prev-active (- id 1)))

            (if (and (= active 1) (not-eq prev 1))
                (print (str-merge "Slave " (str-from-n id "%d") " connected")))

            (if (and (= active 0) (= prev 1)) {
                (print (str-merge "Slave " (str-from-n id "%d") " disconnected"))
                (stop-all-balancing)
                ; Ask remaining active slaves to stop and beep shutdown pattern.
                (var alert-id 1)
                (loopwhile (<= alert-id max-id) {
                    (if (and (not-eq alert-id id) (master-slave-active? alert-id))
                        (master-send-balance alert-id 0 0 0x04))
                    (setq alert-id (+ alert-id 1))
                })
            })

            (setix prev-active (- id 1) active)
            (setq id (+ id 1))
        })
    })

    (setq loop-cnt (+ loop-cnt 1))
    (sleep 0.05)
})
