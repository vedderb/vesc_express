(def trigger-bal-after-charge false)
(def bal-ok false)
(def is-balancing false)
(def is-charging false)
(def charge-ok false)
(def charge-dis-ts (systime))

(def cell-num (+ (bms-get-param 'cells_ic1) (bms-get-param 'cells_ic2)))

(def rtc-val-magic 113)

(def rtc-val '(
        (wakeup-cnt . 0)
        (c-min . 3.5)
        (soc . 0.5)
        (charge-fault . false)
))

(if (= (bufget-u8 (rtc-data) 900) rtc-val-magic) {
      (var tmp (unflatten (rtc-data)))
      (if tmp (setq rtc-val tmp))
})

(defun save-rtc-val () {
        (setassoc rtc-val 'wakeup-cnt (+ (assoc rtc-val 'wakeup-cnt) 1))
        (var tmp (flatten rtc-val))
        (bufcpy (rtc-data) 0 tmp 0 (buflen tmp))
        (bufset-u8 (rtc-data) 900 rtc-val-magic)
})

(defun test-chg (samples) {
        ; Many chargers "pulse" before starting, try to catch a pulse
        (var vchg 0.0)
        (looprange i 0 samples {
                (if (> i 0) (sleep 0.01))
                (var v (bms-get-vchg))
                (if (> v vchg) (setq vchg v))
        })

        (var res (> vchg (* (bms-get-param 'vc_charge_min) cell-num)))

        (if res (setq charge-dis-ts (systime)))

        res
})

(defun is-connected () (or (connected-wifi) (connected-usb) (connected-ble)))

{
    (var do-sleep true)
    (var sleep-early true)

    (if (= (bms-get-btn) 1) {
            (setq do-sleep false)
    })

    (if (test-chg 5)
        (if (not (assoc rtc-val 'charge-fault)) {
                (setq do-sleep false)
        })

        ; Reset charge fault when the charger is not connected at boot
        (setassoc rtc-val 'charge-fault false)
    )

    (if (is-connected) {
            (setq do-sleep false)
    })

    ; Every 1000 startupts we do some additional checks. That allows
    ; adjusting the sleep time based on the SoC.
    (if (= (mod (assoc rtc-val 'wakeup-cnt) 1000) 0) {
            (setq sleep-early false)
    })

    (if (and do-sleep sleep-early) {
            (save-rtc-val)

            (if (< (assoc rtc-val 'soc) 0.05)
                {
                    (bms-set-btn-wakeup-state -1)
                    (sleep-deep 100.0)
                }
                {
                    (bms-set-btn-wakeup-state 1)
                    (sleep-deep 10.0)
                }
            )
    })

    (loopwhile (not (main-init-done)) (sleep 0.1))
    (loopwhile (not (bms-init (bms-get-param 'cells_ic1) (bms-get-param 'cells_ic2))) (sleep 1))

    ; For some reason a second init is sometimes needed to get the BQs started reliably. We
    ; should try to figure out what is going on here...
    (loopwhile (not (bms-init (bms-get-param 'cells_ic1) (bms-get-param 'cells_ic2))) (sleep 1))

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

    (setassoc rtc-val 'c-min c-min)
    (setassoc rtc-val 'soc soc)

    (var t-sorted (sort < (bms-get-temps)))
    (var t-min (ix t-sorted 0))
    (var t-max (ix t-sorted -1))

    (setq charge-ok (and
            (< c-max (bms-get-param 'vc_charge_end))
            (> c-min (bms-get-param 'vc_charge_min))
            (< t-max (bms-get-param 't_charge_max))
            (not (assoc rtc-val 'charge-fault))
            ;(> t-min (bms-get-param 't_charge_min))
    ))

    (var ichg 0.0)
    (if (and (test-chg 10) charge-ok) {
            (bms-set-chg 1)
            (setq is-charging true)

            ; Wait at most 3 seconds for charging current
            (looprange i 0 30 {
                    (sleep 0.1)
                    (setq ichg (- (bms-get-current)))
                    (if (> ichg (bms-get-param 'min_charge_current)) {
                            (setq do-sleep false)
                            (setq trigger-bal-after-charge true)
                            (break)
                    })
            })
    })

    (if (and (< soc 0.05) (not trigger-bal-after-charge)) (setq do-sleep true))

    ;    (sleep 5)
    ;    (print v-cells)
    ;    (print soc)
    ;    (print ichg)
    ;    (print do-sleep)
    ;    (print tries)

    (if do-sleep {
            (bms-sleep)
            (save-rtc-val)

            (if (< (assoc rtc-val 'soc) 0.05)
                {
                    (bms-set-btn-wakeup-state -1)
                    (sleep-deep 100.0)
                }
                {
                    (bms-set-btn-wakeup-state 1)
                    (sleep-deep 10.0)
                }
            )
    })
}

; === TODO===
;
; = Sleep =
;  - Go to sleep when key is left on
;  - [OK] ]Disable wifi after sleep and enable when
;    staying on to save power
;  - [OK] Shorter charge check when waking up so that sleep
;    can be entered faster.
; = Charge control =
;  - [OK] Max current and latch fault so that charger needs to be unplugged before retrying
;  - T min is disabled now. Figure out when temp
;    sensors are missing.
;  - [OK] Charge voltage detection.
; = Balancing =
;  - Balance modes
;  - Limit channel number when getting warm

(def psw-state false)
(def psw-error false)
(def is-bal false)
(def temp-num (length (bms-get-temps)))
(def vtot 0.0)
(def vout 0.0)
(def vchg 0.0)
(def iout 0.0)
(def soc -1.0)
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

; Restore settings if version number does not match
; as that probably means something else is in eeprom
(if (not-eq (read-setting 'ver-code) settings-version) (restore-settings))

(def ah-cnt (read-setting 'ah-cnt))
(def wh-cnt (read-setting 'wh-cnt))
(def ah-chg-tot (read-setting 'ah-chg-tot))
(def wh-chg-tot (read-setting 'wh-chg-tot))
(def ah-dis-tot (read-setting 'ah-dis-tot))
(def wh-dis-tot (read-setting 'wh-dis-tot))

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

        (var t-start (systime))
        (var v-start (bms-get-vout))
        (bms-set-pchg 1)

        (loopwhile (< (secs-since t-start) 3.0) {
                (if (< (- v-tot (bms-get-vout)) 10.0) {
                        (setq res t)
                        (print (str-from-n (* (secs-since t-start) 1000.0) "PCHG T: %.1f ms"))
                        (break)
                })
                (sleep 0.005)
        })

        (if res
            {
                (bms-set-out 1)
                (var diff (- (bms-get-vout) v-start))
                (var cap-est (/ (* (secs-since t-start) 0.9) diff))
                (print (list "Cap est: " (* cap-est 1000.0) "mF"))
            }
            (print "Timed out, make sure that there is a load on the output and no short!")
        )

        (bms-set-pchg 0)

        (def psw-state true)

        res
})

(defun psw-off () {
        (bms-set-pchg 0)
        (bms-set-out 0)
        (def psw-state false)
})

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
            ;            ((? a) (print a))
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

(def t-last (systime))
(def charge-ts (systime))

(defun set-chg (chg) {
        (if chg
            {
                (bms-set-chg 1)
                (setq is-charging true)
            }
            {
                (bms-set-chg 0)
                (setq is-charging false)

                ; Trigger balancing when charging ends and the charge
                ; has been ongoing for at least 10 seconds.
                (if (> (secs-since charge-ts) 10.0) {
                        (setq trigger-bal-after-charge true)
                })

                (setq charge-ts (systime))
            }
        )
})

; Power switch
(loopwhile-thd 100 t {
        (if (and (= (bms-get-btn) 0) psw-state) {
                (psw-off)
                (setq psw-error false)
        })

        (if (and (= (bms-get-btn) 1) (not psw-state) (not psw-error)) {
                (setq psw-error (not (psw-on)))
        })

        (sleep 0.2)
})

(loopwhile-thd 200 t {
        (var v-cells (bms-get-vcells))

        (var t-sorted (sort < (bms-get-temps)))
        (var t-min (ix t-sorted 0))
        (var t-max (ix t-sorted -1))

        (var c-sorted (sort < v-cells))
        (var c-min (ix c-sorted 0))
        (var c-max (ix c-sorted -1))

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

        ;;; Charge control

        (setq charge-ok (and
                (< c-max (if is-charging
                        (bms-get-param 'vc_charge_end)
                        (bms-get-param 'vc_charge_start)
                ))
                (> c-min (bms-get-param 'vc_charge_min))
                (< t-max (bms-get-param 't_charge_max))
                ;(> t-min (bms-get-param 't_charge_min))
                chg-allowed
                (not (assoc rtc-val 'charge-fault))
        ))

        ; If charging is enabled and maximum charge current is exceeded a charge fault is latched
        (if (and is-charging (> (- iout) (bms-get-param 'max_charge_current))) {
                (setq charge-ok false)
                (setassoc rtc-val 'charge-fault true)
        })

        ; Reset latched charge fault after disconnecting charger for 5s
        (if (and (assoc rtc-val 'charge-fault) (> (secs-since charge-dis-ts) 5.0)) {
                (setassoc rtc-val 'charge-fault false)
        })

        (if (and (test-chg 1) charge-ok)
            {
                (if (< (secs-since charge-ts) 2.0)
                    (set-chg true)
                    (set-chg (> (- iout) (bms-get-param 'min_charge_current)))
                )
            }
            {
                (set-chg nil)
            }
        )

        ;;; Sleep

        (setassoc rtc-val 'c-min c-min)
        (setassoc rtc-val 'soc soc)

        ; Measure time without current
        (if (> (abs iout) (bms-get-param 'min_current_sleep))
            (setq i-zero-time 0.0)
            (setq i-zero-time (+ i-zero-time dt))
        )

        ; Go to sleep when button is off, not balancing and not connected
        (if (and (= (bms-get-btn) 0) (> i-zero-time 1.0) (not is-balancing) (not (is-connected))) {
                (sleep 0.1)
                (if (= (bms-get-btn) 0) {
                        (save-rtc-val)
                        (save-settings)
                        (bms-sleep)
                        (bms-set-btn-wakeup-state 1)
                        (sleep-deep 10)
                })
        })

        ; Always go to sleep when SOC is too low
        (if (and (< soc 0.05) (> i-zero-time 1.0)) {
                ; Sleep longer and do not use the key to wake up when
                ; almost empty
                (save-rtc-val)
                (save-settings)
                (bms-sleep)
                (bms-set-btn-wakeup-state -1)
                (sleep-deep 100.0)
        })

        (sleep 0.1)
})

;;; Balancing
(loopwhile-thd 200 t {
        ; Disable balancing and wait for a bit to get clean
        ; measurements
        (looprange i 0 cell-num (bms-set-bal i 0))
        (sleep 2.0)

        (var v-cells (bms-get-vcells))

        (var cells-sorted (sort (fn (x y) (> (ix x 1) (ix y 1)))
            (map (fn (x) (list x (ix v-cells x))) (range cell-num)))
        )

        (var c-min (second (ix cells-sorted -1)))
        (var c-max (second (ix cells-sorted 0)))

        (var t-sorted (sort < (bms-get-temps)))
        (var t-min (ix t-sorted 0))
        (var t-max (ix t-sorted -1))

        (if trigger-bal-after-charge (setq bal-ok true))

        (if (> (* (abs iout) (if is-balancing 0.8 1.0)) (bms-get-param 'balance_max_current)) {
                (setq bal-ok false)
        })

        (if (< c-min (bms-get-param 'vc_balance_min)) {
                (setq bal-ok false)
        })

        (if (> t-max (bms-get-param 't_bal_lim_start)) {
                (setq bal-ok false)
        })

        (if bal-ok (setq trigger-bal-after-charge false))

        (if bal-ok {
                (var bal-chs (map (fn (x) 0) (range cell-num)))
                (var ch-cnt 0)

                (loopforeach c cells-sorted {
                        (var n-cell (first c))
                        (var v-cell (second c))

                        (if (and
                                (> (- v-cell c-min)
                                    (if is-balancing
                                        (bms-get-param 'vc_balance_end)
                                        (bms-get-param 'vc_balance_start)
                                ))
                                ; Do not balance adjacent cells
                                (or (eq n-cell 0) (= (ix bal-chs (- n-cell 1)) 0))
                                (or (eq n-cell (- cell-num 1)) (= (ix bal-chs (+ n-cell 1)) 0))
                            )
                            {
                                (setix bal-chs n-cell 1)
                                (setq ch-cnt (+ ch-cnt 1))
                        })

                        (if (>= ch-cnt (bms-get-param 'max_bal_ch)) (break))
                })

                (looprange i 0 cell-num (bms-set-bal i (ix bal-chs i)))

                (setq is-balancing (> ch-cnt 0))
        })

        (if (not bal-ok) {
                (looprange i 0 cell-num (bms-set-bal i 0))
                (setq is-balancing false)
        })

        (var bal-ok-before bal-ok)
        (looprange i 0 15 {
                (if (not (eq bal-ok-before bal-ok)) (break))
                (sleep 1.0)
        })
})
