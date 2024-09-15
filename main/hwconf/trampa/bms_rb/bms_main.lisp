(bms-init)
(def t-init-done (systime))

(defun is-connected () (or (connected-wifi) (connected-usb) (connected-ble)))

(def charge-at-boot false)
(def bal-ok false)
(def is-balancing false)

(defun go-to-sleep () {
        (save-settings)
        (bms-sleep)
        (bms-set-btn-wakeup-state 1)
        (sleep-deep 6)
})

(defun start-fun () {
        (var do-sleep true)

        (if (= (bms-get-btn) 1) {
                (setq do-sleep false)
        })

        (var soc -2.0)
        (var v-cells nil)
        (var tries 0)
        (var c-min 0.0)
        (var c-max 0.0)

        ; It takes a few reads to get valid voltages the first time
        (loopwhile (< soc -1.5) {
                (setq v-cells (bms-get-vcells))
                (var v-sorted (sort < v-cells))
                (setq c-min (ix v-sorted 0))
                (setq c-max (ix v-sorted -1))
                (setq soc (/ (- c-min 3.1) 1.1))
                (setq tries (+ tries 1))
                (sleep 0.1)
        })

        ; The charger "pulses" before starting, try to catch a pulse
        (var vchg 0.0)
        (looprange i 0 10 {
                (var v (bms-get-vchg))
                (if (> v vchg) (setq vchg v))
                (sleep 0.02)
        })

        (var t-sorted (sort < (bms-get-temps)))
        (var t-min (ix t-sorted 0))
        (var t-max (ix t-sorted -1))

        (var charge-ok (and
                (< c-max (bms-get-param 'vc_charge_end))
                (> c-min (bms-get-param 'vc_charge_min))
                (< t-max (bms-get-param 't_charge_max))
                (> t-min (bms-get-param 't_charge_min))
        ))

        (var ichg 0.0)
        (if (and (> vchg 53.0) charge-ok) {
                (bms-set-chg 1)
                (sleep 0.5)
                (setq ichg (- (bms-get-current)))
                (if (> ichg 0.3) {
                        (setq do-sleep false)
                        (setq charge-at-boot true)
                })
        })

        (if (and (< soc 0.05) (not charge-at-boot)) (setq do-sleep true))

        ; (sleep 5)
        ; (print v-cells)
        ; (print soc)
        ; (print ichg)
        ; (print do-sleep)
        ; (print tries)

        (if (and do-sleep (not (is-connected))) {
                (bms-sleep)

                (if (< soc 0.05)
                    {
                        ; Sleep longer and do not use the key to wake up when
                        ; almost empty
                        (bms-set-btn-wakeup-state -1)
                        (sleep-deep 60.0)
                    }
                    {
                        (bms-set-btn-wakeup-state 1)
                        (sleep-deep 6.0)
                    }
                )
        })
})

(loopwhile t {
        (spawn-trap "start-fun" start-fun)
        (recv
            ((exit-error (? tid) (? e))
                (print (str-merge "Start fun error: " (to-str e)))
            )
            ((exit-ok (? tid) (? v)) (break))
        )
        (sleep 1.0)
})

(def psw-state false)
(def psw-error false)
(def is-bal false)
(def cell-num (length (bms-get-vcells)))
(def temp-num (length (bms-get-temps)))
(def vtot 0.0)
(def vout 0.0)
(def vchg 0.0)
(def iout 0.0)
(def soc -1.0)
(def c-min 0.0)
(def c-max 0.0)
(def i-zero-time 0.0)
(def chg-allowed true)

; Persistent settings
; Format: (label . (offset type))
(def eeprom-addrs '(
        (ver-code    . (0 i))
        (ah-cnt      . (1 f))
        (wh-cnt      . (2 f))
        (ah-chg-tot  . (3 f))
        (wh-chg-tot  . (4 f))
        (ah-dis-tot  . (5 f))
        (wh-dis-tot  . (6 f))
))

; Settings version
(def settings-version 239i32)

(defun read-setting (name)
    (let (
            (addr (first (assoc eeprom-addrs name)))
            (type (second (assoc eeprom-addrs name)))
        )
        (cond
            ((eq type 'i) (eeprom-read-i addr))
            ((eq type 'f) (eeprom-read-f addr))
            ((eq type 'b) (!= (eeprom-read-i addr) 0))
)))

(defun write-setting (name val)
    (let (
            (addr (first (assoc eeprom-addrs name)))
            (type (second (assoc eeprom-addrs name)))
        )
        (cond
            ((eq type 'i) (eeprom-store-i addr val))
            ((eq type 'f) (eeprom-store-f addr val))
            ((eq type 'b) (eeprom-store-i addr (if val 1 0)))
)))

(defun restore-settings ()
    (progn
        (write-setting 'ah-cnt 0.0)
        (write-setting 'wh-cnt 0.0)
        (write-setting 'ah-chg-tot 0.0)
        (write-setting 'wh-chg-tot 0.0)
        (write-setting 'ah-dis-tot 0.0)
        (write-setting 'wh-dis-tot 0.0)
        (write-setting 'ver-code settings-version)
))

(defun save-settings () {
        (write-setting 'ah-cnt ah-cnt)
        (write-setting 'wh-cnt wh-cnt)
        (write-setting 'ah-chg-tot ah-chg-tot)
        (write-setting 'wh-chg-tot wh-chg-tot)
        (write-setting 'ah-dis-tot ah-dis-tot)
        (write-setting 'wh-dis-tot wh-dis-tot)
})

(defun lpf (val sample tc)
    (- val (* tc (- val sample)))
)

(defun truncate (n min max)
    (if (< n min)
        min
        (if (> n max)
            max
            n
)))

(defun psw-on () {
        (var res false)
        (var v-tot (apply + (bms-get-vcells)))

        (bms-set-pchg 0)
        (bms-set-out 0)

        ; Wait for the charge pump to finish
        (loopwhile (< (secs-since t-init-done) 2.0) (sleep 0.1))

        (var t-start (systime))
        (bms-set-pchg 1)

        (loopwhile (< (secs-since t-start) 0.2) {
                (if (> (bms-get-vout) (* v-tot 0.80)) {
                        (setq res t)
                        (print (str-from-n (* (secs-since t-start) 1000.0) "PCHG T: %.1f ms"))
                        (break)
                })
                (sleep 0.01)
        })

        (def pchg-t (secs-since t-start))

        (if res
            (bms-set-out 1)
            {
                (bms-set-pchg 0)
                (print "Timed out, make sure that there is no short on the output!")
            }
        )

        (def psw-state true)

        res
})

(defun psw-off () {
        (bms-set-pchg 0)
        (bms-set-out 0)
        (def psw-state false)
})

; Restore settings if version number does not match
; as that probably means something else is in eeprom
(if (not-eq (read-setting 'ver-code) settings-version) (restore-settings))

(def ah-cnt (read-setting 'ah-cnt))
(def wh-cnt (read-setting 'wh-cnt))
(def ah-chg-tot (read-setting 'ah-chg-tot))
(def wh-chg-tot (read-setting 'wh-chg-tot))
(def ah-dis-tot (read-setting 'ah-dis-tot))
(def wh-dis-tot (read-setting 'wh-dis-tot))

(defun event-handler ()
    (loopwhile t
        (recv
            ((event-bms-chg-allow (? allow)) (setq chg-allowed (= allow 1)))
            ((event-bms-reset-cnt (? ah) (? wh)) {
                    (if (= ah 1) (setq ah-cnt 0.0))
                    (if (= wh 1) (setq wh-cnt 0.0))
            })
            ((event-bms-force-bal (? v)) (if (= v 1)
                    (setq bal-ok true)
                    (setq bal-ok false)
            ))
            (_ nil)
            ;((? a) (print a))
)))

(event-register-handler (spawn event-handler))
(event-enable 'event-bms-chg-allow)
(event-enable 'event-bms-bal-ovr)
(event-enable 'event-bms-reset-cnt)
(event-enable 'event-bms-force-bal)
(event-enable 'event-bms-zero-ofs)

(set-bms-val 'bms-cell-num cell-num)
(set-bms-val 'bms-temp-adc-num temp-num)
(set-bms-val 'bms-can-id (can-local-id))

(defun main-thd () {
        (var t-last (systime))
        (loopwhile t {
                (var v-cells (bms-get-vcells))

                (var t-sorted (sort < (bms-get-temps)))
                (var t-min (ix t-sorted 0))
                (var t-max (ix t-sorted -1))

                (setq vtot (apply + v-cells))
                (setq vout (bms-get-vout))
                (setq vchg (bms-get-vchg))
                (setq iout (bms-get-current))

                (looprange i 0 cell-num {
                        (set-bms-val 'bms-v-cell i (ix v-cells i))
                        (set-bms-val 'bms-bal-state i (bms-get-bal i))
                })

                (looprange i 0 temp-num {
                        (set-bms-val 'bms-temps-adc i (ix (bms-get-temps) i))
                })

                (var cells-sorted (sort (fn (x y) (> (ix x 1) (ix y 1)))
                    (map (fn (x) (list x (ix v-cells x))) (range cell-num)))
                )
                (setq c-min (second (ix cells-sorted -1)))
                (setq c-max (second (ix cells-sorted 0)))

                (if (>= soc 0.0)
                    (setq soc (lpf soc (/ (- c-min 3.1) 1.1) (* 100.0 (bms-get-param 'soc_filter_const))))
                    (setq soc (/ (- c-min 3.1) 1.1))
                )

                (var dt (secs-since t-last))
                (setq t-last (systime))

                ; Ah and Wh cnt
                (if (> (abs iout) (bms-get-param 'min_current_ah_wh_cnt)) {
                        (var ah (* iout (/ dt 3600.0)))
                        (var wh (* ah vtot))

                        (setq ah-cnt (+ ah-cnt ah))
                        (setq wh-cnt (+ wh-cnt wh))

                        (if (> iout 0.0)
                            {
                                (setq ah-dis-tot (+ ah-dis-tot ah))
                                (setq wh-dis-tot (+ wh-dis-tot wh))
                            }
                            {
                                (setq ah-chg-tot (- ah-chg-tot ah))
                                (setq wh-chg-tot (- wh-chg-tot wh))
                            }
                        )
                })

                (set-bms-val 'bms-v-tot vtot)
                (set-bms-val 'bms-v-charge vchg)
                (set-bms-val 'bms-i-in-ic iout)
                (set-bms-val 'bms-temp-ic (ix (bms-get-temps) 0))
                (set-bms-val 'bms-temp-cell-max t-max)
                (set-bms-val 'bms-soc soc)
                (set-bms-val 'bms-soh 1.0)
                (set-bms-val 'bms-ah-cnt ah-cnt)
                (set-bms-val 'bms-wh-cnt wh-cnt)
                (set-bms-val 'bms-ah-cnt-chg-total ah-chg-tot)
                (set-bms-val 'bms-wh-cnt-chg-total wh-chg-tot)
                (set-bms-val 'bms-ah-cnt-dis-total ah-dis-tot)
                (set-bms-val 'bms-wh-cnt-dis-total wh-dis-tot)

                (send-bms-can)

                ; Power switch
                (if (and (= (bms-get-btn) 0) psw-state) {
                        (psw-off)
                        (setq psw-error false)
                })
                (if (and (= (bms-get-btn) 1) (not psw-state) (not psw-error)) {
                        (setq psw-error (not (psw-on)))
                })

                ;;; Charge control

                (var charge-ok (and
                        (< c-max (bms-get-param 'vc_charge_end))
                        (> c-min (bms-get-param 'vc_charge_min))
                        (< t-max (bms-get-param 't_charge_max))
                        (> t-min (bms-get-param 't_charge_min))
                        chg-allowed
                ))
                (bms-set-chg (if charge-ok 1 0))

                ;;; Balancing

                (if charge-at-boot (setq bal-ok true))

                (if (> (* (abs iout) (if is-balancing 0.8 1.0)) (bms-get-param 'balance_max_current)) {
                        (setq bal-ok false)
                })

                (if (< c-min (bms-get-param 'vc_balance_min)) {
                        (setq bal-ok false)
                })

                (if (> t-max (bms-get-param 't_bal_lim_start)) {
                        (setq bal-ok false)
                })

                (if bal-ok (setq charge-at-boot false))

                (if bal-ok {
                        (loopforeach c cells-sorted {
                                (if (> (- (second c) c-min) (bms-get-param 'vc_balance_start)) {
                                        (bms-set-bal (first c) 1)
                                })

                                (if (< (- (second c) c-min) (bms-get-param 'vc_balance_end)) {
                                        (bms-set-bal (first c) 0)
                                })
                        })

                        ; Limit number of channels
                        (var ch-cnt 0)
                        (looprange i 0 cell-num {
                                (if (and (<= ch-cnt (bms-get-param 'max_bal_ch)) (= (bms-get-bal i) 1))
                                    (setq ch-cnt (+ ch-cnt 1))
                                )

                                (if (> ch-cnt (bms-get-param 'max_bal_ch)) (bms-set-bal i 0))
                        })

                        (setq is-balancing (> ch-cnt 0))
                })

                (if (not bal-ok) {
                        (looprange i 0 cell-num (bms-set-bal i 0))
                        (setq is-balancing false)
                })

                ;;; Sleep

                ; Measure time without current
                (if (or
                        (> (abs iout) (bms-get-param 'min_current_sleep))
                        (> (- iout) (bms-get-param 'min_charge_current))
                    )
                    (setq i-zero-time 0.0)
                    (setq i-zero-time (+ i-zero-time dt))
                )

                ; Go to sleep when button is off, not balancing and not connected
                (if (and (= (bms-get-btn) 0) (> i-zero-time 1.0) (not is-balancing) (not (is-connected))) {
                        (sleep 0.1)
                        (if (= (bms-get-btn) 0) (go-to-sleep))
                })

                ; Go to sleep when SOC is too low
                (if (and (< soc 0.02) (> i-zero-time 1.0)) {
                        (go-to-sleep)
                })

                (sleep 0.1)
        })
})

(loopwhile t {
        (spawn-trap "main-thd" main-thd)
        (recv
            ((exit-error (? tid) (? e))
                (print (str-merge "main-thread error: " (to-str e)))
            )
            ((exit-ok (? tid) (? v)) 'ok)
        )
        (sleep 1.0)
})

; === TODO===
;
; = Sleep =
;  - [OK] With key
;  - [OK] Wake up from charger
;  - Go to sleep when key is left on
;  - [OK] Go to sleep when battery too low
;  - [OK] When not balancing
;  - [OK] Much longer poll time and no key when soc low
; = Charge control =
;  - [OK] Temperature protection
;  - [OK] Max cell voltage
;  - [OK] Chg allow state
;  - Max current
; = Balancing =
;  - [OK] Hysteresis
;  - [OK] Max channels
;  - Balance modes
; [OK] Ah cnt
; [OK] Wh cnt
; [OK] Save cnt in sleep
; PSW error when voltage rises too fast
