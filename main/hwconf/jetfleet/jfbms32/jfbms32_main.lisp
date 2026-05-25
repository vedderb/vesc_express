;;;;;;;;; User Settings ;;;;;;;;;

; TODO: Move into config?

; Wait this long for charger to start charging
(def charger-max-delay 10.0)
(def current-scale 0.9675) ; Meter calibration: 1.19A actual / 1.23A reported
(def sleep-unblock-en true) ; Enable automatic sleep unblocking
(def app-wdt-timeout 120) ; Seconds. Set to 0 to disable the app watchdog.
(def user-beeps-en true) ; Enable normal user feedback beeps.

;;;;;;;;; End User Settings ;;;;;;;;;

; State
(def trigger-bal-after-charge false)
(def bal-ok false)
(def is-balancing false)
(def is-charging false)
(def charge-ok false)
(def charge-complete false)
(def charge-complete-msg false)
(def charger-detected-prev false)
(def c-min 0.0)
(def c-max 0.0)
(def t-min 0.0)
(def t-max 0.0)
(def t-mos 0.0)
(def t-ic 0.0)
(def charge-wakeup false)
(def init-done false)

(def chg-status "")
(def bq-status "")
(def bq-status-latched "")
(def bal-status "")
(def bq-safety-a1 0)
(def bq-safety-a2 0)
(def bq-scd-latched false)
(def bq-scd-recovery-armed false)
(def bal-off-failed false)
(def fail-close-failed false)

(def shutdown-reason-unknown 0)
(def shutdown-reason-timer 1)
(def shutdown-reason-low-soc-start 2)
(def shutdown-reason-low-soc-main 3)
(def shutdown-reason-app 4)

(def last-bq-init-attempts 0)
(def last-bq-detect-08 0)
(def last-bq-detect-10 0)
(def last-bq-wake-stage 0)
(def last-bq-wake-time-s 0)

(def rtc-val '(
        (wakeup-cnt . 0)
        (sleep-enter-time-s . 0)
        (sleep-total-time-s . 0)
        (c-min . 3.5)
        (c-max . 3.5)
        (v-tot . 50.0)
        (soc . 0.5)
        (charge-fault . false)
        (updated . false)
        (last-bq-init-attempts . 0)
        (last-bq-detect-08 . 0)
        (last-bq-detect-10 . 0)
        (last-bq-wake-stage . 0)
        (last-bq-wake-time-s . 0)
))

(def is-bal false)
(def vtot 0.0)
(def vout 0.0)
(def vt-vchg 0.0)
(def iout 0.0)
(def soc -1.0)
(def i-zero-time 0.0)
(def chg-allowed true)
(def current-zero-offset 0.0)
(def com-force-on false)
(def com-mutex (mutex-create))
(def buz-mutex (mutex-create))
(def did-crash false)
(def crash-cnt 0)

@const-start

;;; Hack until problem is found ;;;
; Pre-load all functions that are loaded with the dynamic loader. This will
; make them end up in the image and there is no need to load them dynamically.

str-merge
foldl
foldr
zipwith
filter
str-cmp-asc
str-cmp-dsc
second
third
abs

defun
defunret
defmacro
loopfor
loopwhile
looprange
loopforeach
loopwhile-thd

;;; Hack End ;;;

; Current inverted compared to stock FW
(defun bms-current-raw () (* (bms-get-current) -1.0 current-scale))
(defun bms-current () (- (bms-current-raw) current-zero-offset))

(defun beep (times dt) {
        (mutex-lock buz-mutex)

        (loopwhile (> times 0) {
                (pwm-set-duty 0.01 0)
                (sleep dt)
                (pwm-set-duty 0.0 0)
                (sleep dt)
                (setq times (- times 1))
        })

        (mutex-unlock buz-mutex)
})

(defun user-beep (times dt) {
        (if user-beeps-en (beep times dt))
})

(defun beeper-on () {
    (loopwhile t
        (pwm-set-duty 0.01 0)
        (sleep 5)
    )
})

(def rtc-val-magic 124)

; If in deepsleep, this will return 4
; (bms-direct-cmd 1 0x00)

; Exit deepsleep
; (bms-subcmd-cmdonly 1 0x000e)

(defun init-hw () {
        (var attempts 0)

        (loopwhile (not (bms-init (bms-get-param 'cells_ic1) (bms-get-param 'cells_ic2))) {
                (setq attempts (+ attempts 1))
                (record-bq-wake-debug 0 attempts)

                ; Fault alarm: 6 fast marker beeps, then a stage code.
                ; Rate-limit it so a missing BQ warns loudly without wasting
                ; more pack energy on a continuous buzzer.
                (if (or (= attempts 1) (= (mod attempts 10) 0))
                        (bq-wake-debug-beep last-bq-wake-stage)
                )

                (bq-exit-deepsleep-all)
                (wdt-reset)
                (sleep (if (< attempts 5) 1.0 3.0))
        })

        (var st1 (bq-status 1))
        (if (= st1 -1) {
                (record-bq-wake-debug 5 attempts)
                (bq-wake-debug-beep last-bq-wake-stage)
                (exit-error 0)
        })

        (if (= st1 4) {
                (var tries 0)
                (loopwhile (and (= st1 4) (< tries 20)) {
                        (bq-exit-deepsleep-all)
                        (wdt-reset)
                        (sleep 0.05)
                        (setq st1 (bq-status 1))
                        (setq tries (+ tries 1))
                })

                (if (= st1 4) {
                        (record-bq-wake-debug 6 attempts)
                        (bq-wake-debug-beep last-bq-wake-stage)
                        (exit-error 0)
                })
        })

        (if (> (bms-get-param 'cells_ic2) 0) {
                (var st2 (bq-status 2))
                (if (= st2 -1) {
                        (record-bq-wake-debug 7 attempts)
                        (bq-wake-debug-beep last-bq-wake-stage)
                        (exit-error 0)
                })

                (if (= st2 4) {
                        (var tries2 0)
                        (loopwhile (and (= st2 4) (< tries2 20)) {
                                (bq-exit-deepsleep-all)
                                (wdt-reset)
                                (sleep 0.05)
                                (setq st2 (bq-status 2))
                                (setq tries2 (+ tries2 1))
                        })

                        (if (= st2 4) {
                                (record-bq-wake-debug 8 attempts)
                                (bq-wake-debug-beep last-bq-wake-stage)
                                (exit-error 0)
                        })
                })
        })
})

(defun save-rtc-val () {
        (var tmp (flatten rtc-val))
        (bufcpy (rtc-data) 0 tmp 0 (buflen tmp))
        (bufset-u8 (rtc-data) 900 rtc-val-magic)
})

(defun rtc-get (name default)
        (let ((v (assoc rtc-val name)))
                (if v v default)
))

(defun bq-probe-addr (addr)
        (match (trap (eval `(i2c-detect-addr ,addr)))
                ((exit-ok (? a)) (if a 1 0))
                (_ 0)
        )
)

(defun bq-probe-stage ()
        (cond
                ((and (= last-bq-detect-08 0) (= last-bq-detect-10 0)) 1) ; no BQ address responds
                ((and (= last-bq-detect-08 1) (= last-bq-detect-10 0)) 2) ; only default address responds
                ((and (= last-bq-detect-08 0) (= last-bq-detect-10 1)) 3) ; only BQ1 target address responds
                (true 4)                                                   ; both addresses respond
))

(defun bq-wake-stage-name (stage)
        (cond
                ((= stage 1) "no-address")
                ((= stage 2) "only-0x08")
                ((= stage 3) "only-0x10")
                ((= stage 4) "both-addresses")
                ((= stage 5) "bq1-status-read-failed")
                ((= stage 6) "bq1-stuck-deepsleep")
                ((= stage 7) "bq2-status-read-failed")
                ((= stage 8) "bq2-stuck-deepsleep")
                (true "none")
))

(defun bq-wake-debug-beep (stage) {
        ; BQ wake/init issue marker: 6 fast beeps, pause, then stage count.
        (beep 6 0.04)
        (sleep 0.4)
        (beep stage 0.18)
})

(defun bq-exit-deepsleep-all () {
        ; Try both expected BQ logical addresses. Failures are diagnostic only:
        ; the recovery loop below will re-run bms-init until communication is sane.
        (trap (bms-subcmd-cmdonly 1 0x000e))
        (trap (bms-subcmd-cmdonly 1 0x000e))
        (trap (bms-subcmd-cmdonly 2 0x000e))
        (trap (bms-subcmd-cmdonly 2 0x000e))
})

(defun bq-status (ic)
        (match (trap (eval `(bms-direct-cmd ,ic 0x00)))
                ((exit-ok (? a)) a)
                (_ -1)
        )
)

(defun record-bq-wake-debug (stage attempts) {
        (setq last-bq-init-attempts attempts)
        (setq last-bq-detect-08 (bq-probe-addr 0x08))
        (setq last-bq-detect-10 (bq-probe-addr 0x10))
        (setq last-bq-wake-stage (if (= stage 0) (bq-probe-stage) stage))
        (setq last-bq-wake-time-s (get-time-of-day-s))

        (setassoc rtc-val 'last-bq-init-attempts last-bq-init-attempts)
        (setassoc rtc-val 'last-bq-detect-08 last-bq-detect-08)
        (setassoc rtc-val 'last-bq-detect-10 last-bq-detect-10)
        (setassoc rtc-val 'last-bq-wake-stage last-bq-wake-stage)
        (setassoc rtc-val 'last-bq-wake-time-s last-bq-wake-time-s)
        (save-rtc-val)

        (if (or (= attempts 1) (= (mod attempts 10) 0)) {
                (print "BQ wake:" (bq-wake-stage-name last-bq-wake-stage)
                        "attempts" attempts
                        "0x08" last-bq-detect-08
                        "0x10" last-bq-detect-10)
        })
})

(defun sync-bq-wake-debug-globals () {
        (setq last-bq-init-attempts (rtc-get 'last-bq-init-attempts 0))
        (setq last-bq-detect-08 (rtc-get 'last-bq-detect-08 0))
        (setq last-bq-detect-10 (rtc-get 'last-bq-detect-10 0))
        (setq last-bq-wake-stage (rtc-get 'last-bq-wake-stage 0))
        (setq last-bq-wake-time-s (rtc-get 'last-bq-wake-time-s 0))
})

(defun shutdown-reason-name (reason)
        (cond
                ((= reason shutdown-reason-timer) "timer")
                ((= reason shutdown-reason-low-soc-start) "low-soc-start")
                ((= reason shutdown-reason-low-soc-main) "low-soc-main")
                ((= reason shutdown-reason-app) "app")
                (true "unknown")
))

(defun shutdown-reason-beep (reason) {
        ; User feedback: shutdown accepted. The long-beep count also gives a
        ; simple reason code if someone is standing next to the pack.
        (var count (if (> reason 0) reason 5))
        (user-beep count 0.25)
        (sleep 0.8)
        (user-beep 3 0.06)
})

(defun print-shutdown-reason (reason) {
        (print "Shutdown reason:" (shutdown-reason-name reason))
})

(defun prepare-external-wakeup () {
        ; JFBMS32 wakes from external requests on IO2.
        (bms-set-btn-wakeup-state 1)
})

(defun external-wake-active () (= (bms-get-btn) 1))

(defun external-wake-inactive () (= (bms-get-btn) 0))

(defun sleep-duration-s () (* (bms-get-param 'sleep) 3600))

(defun test-chg (samples) {
        ; Many chargers "pulse" before starting, try to catch a pulse
        (var vchg 0.0)
        (looprange i 0 samples {
                (if (> i 0) (sleep 0.01))
                (setq vchg (bms-get-vchg))
                (if (> vchg (bms-get-param 'v_charge_detect)) (break))
        })

        (var res (> vchg (bms-get-param 'v_charge_detect)))

        (if res (setq charge-dis-ts (systime)))

        res
})

(defun truncate (n min max)
    (if (< n min)
        min
        (if (> n max)
            max
            n
)))

; SOC from battery voltage
(defun calc-soc (v-cell) (truncate (/ (- v-cell (bms-get-param 'vc_empty)) (- (bms-get-param 'vc_full) (bms-get-param 'vc_empty))) 0.0 1.0))

(defun valid-pack-reading () (and
        init-done
        (> cell-num 0)
        (> c-min 1.0)
        (> c-max 2.0)
        (< c-min 5.0)
        (< c-max 5.0)
        (>= c-max c-min)
        (> vtot (* cell-num 1.5))
        (>= soc 0.0)
))

; True when a real communication interface is connected.
(defun is-comm-connected () (or (connected-wifi) (connected-usb) (connected-ble)))

; True when VESC Tool is connected or sleep is intentionally blocked.
(defun is-connected () (or (is-comm-connected) (= (bms-get-param 'block_sleep) 1)))
;(defun is-connected () (or (connected-wifi) (connected-ble) (= (bms-get-param 'block_sleep) 1)))

(defunret can-active () {
        (var devs (can-list-devs))
        (if (eq devs nil) (return false))

        (looprange i 1 7 {
                (var res (can-msg-age (first devs) i))
                (if (and res (< res 0.1)) (return true))
        })

        false
})

(defun can-sum-current () {
        (var devs (can-list-devs))
        (var i-sum 0.0)

        (loopforeach d devs {
                (var res (can-msg-age d 4))
                (if (and res (< res 0.1)) (setq i-sum (+ i-sum (canget-current-in d))))
        })

        i-sum
})

; Run expression with communication enabled. Use up to 4 attempts in case of
; glithces from transients. Disable communication again when it is not needed
; to save power.
(defun with-com (expr) {
        (mutex-lock com-mutex)

        (gpio-write 9 0)

        (var res (looprange i 0 4 {
                    (match (trap (eval expr))
                        ((exit-ok (? a)) (break a))
                        (_ (if (= i 3) {
                                    (mutex-unlock com-mutex)
                                    (exit-error 0)
                        }))
                    )
        }))

        (if (and
                (not (can-active))
                (external-wake-inactive)
                (not com-force-on)
                (not (is-connected))
                (not is-charging)
            )
            (gpio-write 9 1)
        )

        (mutex-unlock com-mutex)
        res
})

(defun com-force (en)
    (if en
        {
            (setq com-force-on true)
            (gpio-write 9 0)
        }
        {
            (setq com-force-on false)
        }
    )
)

(defun disable-balancing () {
        (var bal-off-ok false)

        (match (trap (bms-disable-balancing))
                ((exit-ok _) (setq bal-off-ok true))
                (_ {
                        (setq bal-off-ok true)
                        (looprange i 0 cell-num {
                                (match (trap (with-com `(bms-set-bal ,i 0)))
                                        ((exit-ok _) nil)
                                        (_ (setq bal-off-ok false))
                                )
                        })
                })
        )

        (if bal-off-ok
                {
                        (if bal-off-failed (print "Balancing disabled after retry"))
                        (setq bal-off-failed false)
                        (setq is-balancing false)
                        (setq bal-status "")
                }
                {
                        (if (not bal-off-failed) (print "Failed to disable balancing"))
                        (setq bal-off-failed true)
                }
        )

        bal-off-ok
})

(defun fail-close-outputs (clear-bal-trigger) {
        (var close-ok false)

        (match (trap (bms-fail-close-outputs))
                ((exit-ok _) (setq close-ok true))
                (_ {
                        (trap (bms-set-chg 0))
                        (setq close-ok (disable-balancing))
                })
        )

        (setq is-charging false)
        (setq charge-ok false)
        (if clear-bal-trigger (setq trigger-bal-after-charge false))

        (if close-ok
                {
                        (if fail-close-failed (print "BMS fail-close recovered"))
                        (setq fail-close-failed false)
                        (setq bal-off-failed false)
                        (setq is-balancing false)
                        (setq bal-status "")
                }
                {
                        (if (not fail-close-failed) (print "BMS fail-close failed"))
                        (setq fail-close-failed true)
                }
        )

        close-ok
})

(defun balance-safe-now () (and
        (<= (* (abs iout) (if is-balancing 0.8 1.0)) (bms-get-param 'balance_max_current))
        (>= c-min (bms-get-param 'vc_balance_min))
        (<= t-max (bms-get-param 't_bal_max_cell))
        (<= t-ic (bms-get-param 't_bal_max_ic))
))

(defun update-temps () {
        ; Exit if any of the BQs has invalid temperature settings
        (var v1 (bms-read-reg 1 0x92fd 1))
        (var h1 (bms-read-reg 1 0x9300 1))
        (var has-ic2 (> (bms-get-param 'cells_ic2) 0))
        (var v2 (if has-ic2 (with-com '(bms-read-reg 2 0x92fd 1)) 0x3b)) ; default to valid when no IC2
        (var h2 (if has-ic2 (with-com '(bms-read-reg 2 0x9300 1)) 0x3b)) ; default to valid when no IC2

        (if (or
                (and (!= v1 0x3b) (!= v1 0x7b))
                (!= h1 0x3b)
                (and has-ic2 (and (!= v2 0x3b) (!= v2 0x7b)))
                (and has-ic2 (!= h2 0x3b))
            ) {
            (print "Invalid temperature settings, retrying...")
            (sleep 0.01)

            (setq v1 (bms-read-reg 1 0x92fd 1))
            (setq h1 (bms-read-reg 1 0x9300 1))
            (setq v2 (if has-ic2 (with-com '(bms-read-reg 2 0x92fd 1)) 0x3b))
            (setq h2 (if has-ic2 (with-com '(bms-read-reg 2 0x9300 1)) 0x3b))

            (if (or
                    (and (!= v1 0x3b) (!= v1 0x7b))
                    (!= h1 0x3b)
                    (and has-ic2 (and (!= v2 0x3b) (!= v2 0x7b)))
                    (and has-ic2 (!= h2 0x3b))
                ) {
                (print "BQs with invalid temperature settings")
                (exit-error 0)
            })
        })

        (var bms-temps (with-com '(bms-get-temps)))
        (var temp-ext-num (truncate (bms-get-param 'temp_num) 0 4))

        ; bms-temps: BQ1 IC, BQ1 TS1/TS3/ALERT/DCHG, BQ1 HDQ, BQ2 IC, BQ2 HDQ
        (var t-sorted (sort < (map
                    (fn (x) (ix bms-temps (+ x 1)))
                    (range 0 temp-ext-num)
        )))

        ; If all sensors are disabled pretend we are at 24c
        (if (= (length t-sorted) 0) (setq t-sorted '(24)))

        (setq t-min (ix t-sorted 0))
        (setq t-max (ix t-sorted -1))
        (setq t-mos (if (> (ix bms-temps 5) (ix bms-temps 7)) (ix bms-temps 5) (ix bms-temps 7)))
        (setq t-ic  (if (> (ix bms-temps 0) (ix bms-temps 6)) (ix bms-temps 0) (ix bms-temps 6)))

        bms-temps
})

(defun bms-shutdown-failed-alarm () {
    (print "BMS hardware shutdown failed")
    (loopwhile t {
        (fail-close-outputs true)
        (beep 20 0.05)
        (sleep 0.5)
    })
})

(defun bms-shutdown-impl (save-counters reason) {
    (print "BMS shutdown sequence starting")
    (print-shutdown-reason reason)
    (fail-close-outputs true)
    (shutdown-reason-beep reason)

    (setassoc rtc-val 'sleep-enter-time-s (get-time-of-day-s))
    (save-rtc-val)
    (if save-counters (save-settings))

    (match (trap (bms-hw-shutdown))
        ((exit-ok _) nil)
        (_ (bms-shutdown-failed-alarm))
    )
})

(defun bms-shutdown () (bms-shutdown-impl true shutdown-reason-unknown))

(defun bms-shutdown-no-save () (bms-shutdown-impl false shutdown-reason-unknown))

(defun bms-shutdown-timer () (bms-shutdown-impl true shutdown-reason-timer))

(defun bms-shutdown-low-soc-start () (bms-shutdown-impl false shutdown-reason-low-soc-start))

(defun bms-shutdown-low-soc-main () (bms-shutdown-impl true shutdown-reason-low-soc-main))

(defun bms-shutdown-app () (bms-shutdown-impl true shutdown-reason-app))

(defun low-soc-unused () (and
        (valid-pack-reading)
        (< soc 0.05)
        (not trigger-bal-after-charge)
        (external-wake-inactive)
        ; block_sleep is intentionally honored here so a fresh, unconfigured
        ; pack cannot shut itself down before setup is complete.
        (not (is-connected))
        (not (can-active))
))

(defun process-sleep-time () {

        ;; (print "Sleep enter time:" (assoc rtc-val 'sleep-enter-time-s))
        ;; (print "Total Sleep time:" (assoc rtc-val 'sleep-total-time-s))
        ;; (var time-now-s (get-time-of-day-s))
        ;; (print "Time Now:" time-now-s)

        (var source (bms-wakeup-source))

        (cond
                ;((= source 0))
                ; Woke up on GPIO/RTC IO we reset the sleep time
                ((= source 1) {
                        (setassoc rtc-val 'sleep-total-time-s 0)
                })
                ; Woke up on timer
                ((= source 2) {
                        (var time-now-s (get-time-of-day-s))
                        (if (> (assoc rtc-val 'sleep-enter-time-s) 0 ) {
                            (var slept-time (- time-now-s (assoc rtc-val 'sleep-enter-time-s)))
                            (setassoc rtc-val 'sleep-total-time-s (+ (assoc rtc-val 'sleep-total-time-s) slept-time))
                        })

                        (if ( >= (assoc rtc-val 'sleep-total-time-s) (* (bms-get-param 'shutdown) 86400)) ; Shutdown is in DAYS, multiply by 86400
                            (bms-shutdown-timer)
                        )
                })
        )
        (save-rtc-val)
})


(defun start-fun () {
        (setassoc rtc-val 'wakeup-cnt (+ (assoc rtc-val 'wakeup-cnt) 1))

        (var do-sleep true)

        (if (external-wake-active) {
                (setq do-sleep false)
        })

        (var chg-detected (test-chg 5))
        (if (or charge-wakeup chg-detected) {
                (setq do-sleep false)
                (if (not (assoc rtc-val 'charge-fault)) {
                        (setq charge-wakeup true)
                })
        })

        ; Reset charge fault when the charger is not connected at boot
        (if (not chg-detected) {
                (setassoc rtc-val 'charge-fault false)
                (setq charge-complete false)
                (setq charge-complete-msg false)
        })

        (if (is-connected) (setq do-sleep false))
        (if (can-active) (setq do-sleep false))

        (init-hw)

        (user-beep 2 0.1)

        (process-sleep-time)

        (if (can-active) (setq do-sleep false))

        (var soc -2.0)
        (var v-cells nil)
        (var tries 0)

        ; It takes a few reads to get valid voltages the first time
        (loopwhile (< soc -1.5) {
                (setq v-cells (with-com '(bms-get-vcells)))
                (var v-sorted (sort < v-cells))
                (setq c-min (ix v-sorted 0))
                (setq c-max (ix v-sorted -1))
                (setq soc (calc-soc c-min))
                (setq tries (+ tries 1))
                (sleep 0.1)
        })

        (setassoc rtc-val 'c-min c-min)
        (setassoc rtc-val 'c-max c-max)
        (setassoc rtc-val 'v-tot (apply + v-cells))
        (setassoc rtc-val 'soc soc)
        (setassoc rtc-val 'updated true)
        (save-rtc-val)

        (update-temps)

        (setq init-done true)

        (setq charge-ok (and
                (< c-max (bms-get-param 'vc_charge_end))
                (> c-min (bms-get-param 'vc_charge_min))
                (< t-max (bms-get-param 't_charge_max))
                (> t-min (bms-get-param 't_charge_min))
                (< t-mos (bms-get-param 't_charge_max_mos))
                (not (assoc rtc-val 'charge-fault))
                (not charge-complete)
        ))

        (var ichg 0.0)
         (if (and charge-ok charge-wakeup (test-chg 400)) {
                (set-chg true)

                (looprange i 0 (* charger-max-delay 10.0) {
                        (sleep 0.1)
                        (setq ichg (- (bms-current)))
                        (if (> ichg (bms-get-param 'min_charge_current)) {
                                (setq do-sleep false)
                                (setq trigger-bal-after-charge true)
                                (break)
                        })
                })
        })

        (if (low-soc-unused) (bms-shutdown-low-soc-start))

        ;(sleep 5)
        ;(print v-cells)
        ;(print soc)
        ;(print ichg)
        ;(print do-sleep)
        ;(print tries)

        ; Trap bms-sleep failures so a transient mutex
        ; timeout or BQ NAK doesn't put ESP into deep sleep with the BQs
        ; still in ACTIVE mode (top-cell drain). On failure we defer the
        ; sleep-deep call and let the next start-fun retry try again.
        (if do-sleep
            (match (trap (with-com '(do-bms-sleep)))
                ((exit-ok _) {
                    (print "bms-sleep ok, entering sleep-deep")
                    (setassoc rtc-val 'sleep-enter-time-s (get-time-of-day-s))
                    (save-rtc-val)

                    (prepare-external-wakeup)
                    (sleep-deep (sleep-duration-s))
                })
                (_ {
                    (print "bms-sleep FAILED in start-fun -- deferring sleep-deep, will retry")
                    (sleep 1.0)
                })
            )
        )
})

; === TODO===
;
; = Sleep =
;  - Go to sleep when key is left on

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
        (ah-cnt-soc  . (7 f))
))

; Settings version
(def settings-version 242i32)

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

(defun restore-settings () {
        (write-setting 'ah-cnt 0.0)
        (write-setting 'wh-cnt 0.0)
        (write-setting 'ah-chg-tot 0.0)
        (write-setting 'wh-chg-tot 0.0)
        (write-setting 'ah-dis-tot 0.0)
        (write-setting 'wh-dis-tot 0.0)
        (write-setting 'ah-cnt-soc (* (calc-soc c-min) (bms-get-param 'batt_ah)))
        (write-setting 'ver-code settings-version)
})

(defun save-settings () {
        (write-setting 'ah-cnt ah-cnt)
        (write-setting 'wh-cnt wh-cnt)
        (write-setting 'ah-chg-tot ah-chg-tot)
        (write-setting 'wh-chg-tot wh-chg-tot)
        (write-setting 'ah-dis-tot ah-dis-tot)
        (write-setting 'wh-dis-tot wh-dis-tot)
        (write-setting 'ah-cnt-soc ah-cnt-soc)
})

(defun lpf (val sample tc)
    (- val (* tc (- val sample)))
)

(defun status-append (base part)
    (if (> (str-len part) 0)
        (if (> (str-len base) 0)
            (str-merge base " | " part)
            part
        )
        base
))

(defun bq-fault-str-one (prefix flags) {
        (var s "")
        (if (!= (bitwise-and flags 0x80) 0) (setq s (status-append s (str-merge prefix "_SCD"))))
        (if (!= (bitwise-and flags 0x40) 0) (setq s (status-append s (str-merge prefix "_OCD2"))))
        (if (!= (bitwise-and flags 0x20) 0) (setq s (status-append s (str-merge prefix "_OCD1"))))
        (if (!= (bitwise-and flags 0x10) 0) (setq s (status-append s (str-merge prefix "_OCC"))))
        (if (!= (bitwise-and flags 0x08) 0) (setq s (status-append s (str-merge prefix "_COV"))))
        (if (!= (bitwise-and flags 0x04) 0) (setq s (status-append s (str-merge prefix "_CUV"))))
        s
})

(defun update-bq-status () {
        ; SafetyStatusA bits: SCD OCD2 OCD1 OCC COV CUV.
        ; Only BQ1 has the current shunt, so BQ2 cannot produce meaningful
        ; current-protection status on this hardware.
        (setq bq-safety-a1 (bms-direct-cmd 1 0x03))
        (setq bq-safety-a2 0)
        (bq-fault-str-one "BQ1" bq-safety-a1)
})

(defun bq-current-fault-active ()
    (!= (bitwise-and bq-safety-a1 0xB0) 0)
)

(defun bq-scd-fault-active ()
    (!= (bitwise-and bq-safety-a1 0x80) 0)
)

(defun recover-bq-scd-after-disconnect () {
        (if (bq-scd-fault-active) {
                (setq bq-scd-latched true)
                (setq bq-scd-recovery-armed false)
                (set-chg nil)
        })

        ; A charger-side short can also pull Vchg to zero, so absence only arms
        ; recovery. The actual recover waits for a fresh charger-voltage detect.
        (if (and bq-scd-latched (> (secs-since charge-dis-ts) 5.0)) {
                (setq bq-scd-recovery-armed true)
        })
})

(defun recover-bq-scd-on-charger-detect () {
        (if bq-scd-recovery-armed {
                (with-com '(progn
                        (bms-subcmd-cmdonly 1 0x009C) ; SCDL_RECOVER
                ))
                (setassoc rtc-val 'charge-fault false)
                (setq bq-scd-latched false)
                (setq bq-scd-recovery-armed false)
                (setq bq-status-latched "")
                (setq charge-ts (systime))
        })
})

(defun set-chg (chg) {
        (if chg
            {
                (if (not is-charging) (setq charge-ts (systime)))
                (gpio-write 9 0)
                (bms-set-chg 1)
                (setq is-charging true)
            }
            {
                ; Trigger balancing when charging ends and the charge
                ; has been ongoing for at least 10 seconds.
                (if (and is-charging (> (secs-since charge-ts) 10.0)) {
                        (setq trigger-bal-after-charge true)
                })

                (bms-set-chg 0)
                (setq is-charging false)
            }
        )
})

(defun send-can-info () {
        (var buf-canid35 (array-create 8))

        ;(var ah-left (- (bms-get-param 'batt_ah) ah-cnt-soc))
        (var ah-left (* (bms-get-param 'batt_ah) (- 1.0 soc)))
        (var min-left (if (< iout -1.0)
                (* (/ ah-left (- iout)) 60.0)
                0.0
        ))

        (bufset-i16 buf-canid35 0 (* soc 1000)) ; Battery A SOC
        (bufset-u8 buf-canid35 2 (if (> vt-vchg (bms-get-param 'v_charge_detect)) 1 0)) ; Battery A Charging
        (bufset-u16 buf-canid35 3 min-left) ; Battery A Charge Time Minutes
        (bufset-u16 buf-canid35 5 (* (bms-get-param 'batt_ah) 10.0))
        (can-send-sid 35 buf-canid35)

        (send-bms-can)
})

(defun do-bms-sleep () {
   (bms-sleep)
})

(defun main-ctrl () (loopwhile t {
        ; Exit if any of the BQs has fallen asleep
            (if (or
                    (= (bms-direct-cmd 1 0x00) 4)
                    (and (> (bms-get-param 'cells_ic2) 0) (= (with-com '(bms-direct-cmd 2 0x00)) 4))
                )
                (exit-error 0)
            )

            (setq bq-status (update-bq-status))
            (if (> (str-len bq-status) 0) (setq bq-status-latched bq-status))
            (if (and (> (str-len bq-status-latched) 0) (not bq-scd-latched) (> (secs-since charge-dis-ts) 5.0)) {
                    (setq bq-status-latched "")
            })
            (recover-bq-scd-after-disconnect)

            (var v-cells (with-com '(bms-get-vcells)))
            (var bms-temps (update-temps))
        (var temp-ext-num (truncate (bms-get-param 'temp_num) 0 4))

        (var c-sorted (sort < v-cells))
        (setq c-min (ix c-sorted 0))
        (setq c-max (ix c-sorted -1))

        (setq vtot (apply + v-cells))
        (setq vout (with-com '(bms-get-vout)))
        (setq vt-vchg (bms-get-vchg))
        (setq iout (+ (with-com '(bms-current)) (can-sum-current)))

        (if (and is-balancing (not (balance-safe-now))) {
                (setq bal-ok false)
                (disable-balancing)
        })

        (var cell0-report-offset (bms-cell0-report-offset iout))
        (looprange i 0 cell-num {
                (set-bms-val 'bms-v-cell i
                        (- (ix v-cells i) (if (= i 0) cell0-report-offset 0.0))
                )
                (set-bms-val 'bms-bal-state i (bms-get-bal i))
        })

        (set-bms-val 'bms-temp-adc-num (+ 5 temp-ext-num))
        (set-bms-val 'bms-temps-adc 0 t-ic) ; IC
        (set-bms-val 'bms-temps-adc 1 t-min) ; Cell Min
        (set-bms-val 'bms-temps-adc 2 t-max) ; Cell Max
        (set-bms-val 'bms-temps-adc 3 t-mos) ; Mosfet
        (set-bms-val 'bms-temps-adc 4 -300.0) ; Ambient
        (looprange i 0 temp-ext-num {
                (set-bms-val 'bms-temps-adc (+ 5 i) (ix bms-temps (+ i 1)))
        })
        (set-bms-val 'bms-data-version 1)

        (set-bms-val 'bms-v-cell-min c-min)
        (set-bms-val 'bms-v-cell-max c-max)

        (if (= (bms-get-param 'soc_use_ah) 1)
        {
                ; Coulomb counting
                (setq soc (/ ah-cnt-soc (bms-get-param 'batt_ah)))
        }
        {
                (if (>= soc 0.0)
                (setq soc (lpf soc (calc-soc c-min) (* 100.0 (bms-get-param 'soc_filter_const))))
                (setq soc (calc-soc c-min))
                )
        }
        )

        (var dt (secs-since t-last))
        (setq t-last (systime))
        (var t-chg (mod (/ (systime) 1000 60) 255))

        (var ah (* iout (/ dt 3600.0)))
        (setq ah-cnt-soc (truncate (- ah-cnt-soc ah) 0.0 (bms-get-param 'batt_ah)))

        ; Ah and Wh cnt
        (if (> (abs iout) (bms-get-param 'min_current_ah_wh_cnt)) {
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
        (set-bms-val 'bms-v-charge vt-vchg)
        (set-bms-val 'bms-i-in-ic iout)
        (set-bms-val 'bms-temp-ic t-ic)
        (set-bms-val 'bms-temp-cell-max t-max)
        (set-bms-val 'bms-soc soc)
        (set-bms-val 'bms-soh 1.0)
        (set-bms-val 'bms-ah-cnt ah-cnt)
        (set-bms-val 'bms-wh-cnt wh-cnt)
        (set-bms-val 'bms-ah-cnt-chg-total ah-chg-tot)
        (set-bms-val 'bms-wh-cnt-chg-total wh-chg-tot)
        (set-bms-val 'bms-ah-cnt-dis-total ah-dis-tot)
        (set-bms-val 'bms-wh-cnt-dis-total wh-dis-tot)

        (with-com '(send-can-info))

        ;;; Charge control

        ; Once a cell reaches the end voltage, hold charge off until the
        ; charger is disconnected. Without this latch the unloaded cell voltage
        ; can relax below vc_charge_start and make the charger cycle.
        (if (and is-charging (>= c-max (bms-get-param 'vc_charge_end))) {
                (setq charge-complete true)
                (setq charge-complete-msg true)
                (setq trigger-bal-after-charge true)
        })

        (setq charge-ok (and
                (< c-max (if is-charging
                        (bms-get-param 'vc_charge_end)
                        (bms-get-param 'vc_charge_start)
                ))
                (> c-min (bms-get-param 'vc_charge_min))
                (< t-max (bms-get-param 't_charge_max))
                (> t-min (bms-get-param 't_charge_min))
                (< t-mos (bms-get-param 't_charge_max_mos))
                chg-allowed
                (not (assoc rtc-val 'charge-fault))
                (not charge-complete)
                (not bq-scd-latched)
        ))

        (if (bq-current-fault-active) {
                (setq charge-ok false)
        })

        ; If charging is enabled and maximum charge current is exceeded a charge fault is latched
        (if (and is-charging (> (- iout) (bms-get-param 'max_charge_current))) {
                (setq charge-ok false)
                (setassoc rtc-val 'charge-fault true)
        })

        ; Reset latched charge fault after disconnecting charger for 5s
        (if (and (assoc rtc-val 'charge-fault) (> (secs-since charge-dis-ts) 5.0)) {
                (setassoc rtc-val 'charge-fault false)
        })

        ; Allow a new charge session only after the charger has been removed.
        (if (and charge-complete (> (secs-since charge-dis-ts) 5.0)) {
                (setq charge-complete false)
                (setq charge-complete-msg false)
        })

        (setq chg-status
        (cond
                ((assoc rtc-val 'charge-fault) {
                        (setq charge-complete-msg false)
                        "FLT_CHG_OC"
                })
                (charge-complete-msg "CHG_COMPLETE")
                (is-charging {
                        (setq charge-complete-msg false)
                        "CHARGING"
                })
                (true "")
        ))

        ; Set combined BMS status
        (var bq-status-display (if (> (str-len bq-status) 0) bq-status bq-status-latched))
        (var output-fault-status "")
        (if bal-off-failed (setq output-fault-status (status-append output-fault-status "BAL_OFF_FAIL")))
        (if fail-close-failed (setq output-fault-status (status-append output-fault-status "FAIL_CLOSE_FAIL")))
        (if (and
                charge-complete-msg
                (or
                        (> (str-len bq-status-display) 0)
                        (> (str-len output-fault-status) 0)
                )
        ) {
                (setq charge-complete-msg false)
                (setq chg-status "")
        })
        (set-bms-val 'bms-status
                (status-append
                        (status-append (status-append chg-status bq-status-display) bal-status)
                        output-fault-status
                )
        )

        (var charger-detected (test-chg 1))
        (if (and charger-detected (not charger-detected-prev)) {
                (recover-bq-scd-on-charger-detect)
                (setq charge-ts (systime))
        })
        (setq charger-detected-prev charger-detected)

        (if (and charger-detected charge-ok)
        {
                (if (< (secs-since charge-ts) charger-max-delay)
                (set-chg true)
                (set-chg (> (- iout) (bms-get-param 'min_charge_current)))
                )
        }
        {
                (set-chg nil)

                ; Reset coulomb counter when battery is full
                (if (>= c-max (bms-get-param 'vc_charge_start)) {
                        (setq ah-cnt-soc (bms-get-param 'batt_ah))
                        (setq trigger-bal-after-charge true)
                })
        }
        )

        ;;; Sleep

        (setassoc rtc-val 'c-min c-min)
        (setassoc rtc-val 'c-max c-max)
        (setassoc rtc-val 'v-tot vtot)
        (setassoc rtc-val 'soc soc)
        (setassoc rtc-val 'updated true)
        (save-rtc-val)

        ; Measure time without current
        (if (> (abs iout) (bms-get-param 'min_current_sleep))
        (setq i-zero-time 0.0)
        (setq i-zero-time (+ i-zero-time dt))
        )

        ; Go to sleep when button is off, not balancing and not connected
        (if (and (external-wake-inactive) (> i-zero-time 1.0) (not is-balancing) (not (is-connected)) (not (can-active))) {
                (sleep 0.1)
                (if (external-wake-inactive) {
                        (setassoc rtc-val 'sleep-enter-time-s (get-time-of-day-s))
                        (save-rtc-val)
                        (save-settings)
                        ; See start-fun for rationale.
                        (match (trap (with-com '(do-bms-sleep)))
                            ((exit-ok _) {
                                (print "bms-sleep ok, entering sleep-deep")
                                (prepare-external-wakeup)
                                (sleep-deep (sleep-duration-s))
                            })
                            (_ {
                                (print "bms-sleep FAILED in main-ctrl idle path -- deferring sleep-deep, will retry")
                                (sleep 1.0)
                            })
                        )
                })
        })

        ; Set SOC to 0 below 2.9V and not under load.
        (if (and (> i-zero-time 10.0) (<= c-min (bms-get-param 'vc_empty))) {
                (setq ah-cnt-soc 0.0)
        })

        ; Shut down when SOC is too low and nothing external is requesting the BMS.
        (if (and (low-soc-unused) (> i-zero-time 1.0) (<= c-min (bms-get-param 'vc_empty))) {
                (bms-shutdown-low-soc-main)
        })

        (wdt-reset)
        (sleep 0.1)
}))

; Balancing
(defun balance () (loopwhile t {
            ; Disable balancing and wait for a bit to get clean
            ; measurements
            (looprange i 0 cell-num (with-com `(bms-set-bal ,i 0)))
            (sleep 2.0)

            (var v-cells (with-com '(bms-get-vcells)))

            (var cells-sorted (sort (fn (x y) (> (ix x 1) (ix y 1)))
                (map (fn (x) (list x (ix v-cells x))) (range cell-num)))
            )

            (var c-min (second (ix cells-sorted -1)))
            (var c-max (second (ix cells-sorted 0)))

            (if trigger-bal-after-charge (setq bal-ok true))

            (if (> (* (abs iout) (if is-balancing 0.8 1.0)) (bms-get-param 'balance_max_current)) {
                    (setq bal-ok false)
            })

            (if (< c-min (bms-get-param 'vc_balance_min)) {
                    (setq bal-ok false)
            })

            (if (> t-max (bms-get-param 't_bal_max_cell)) {
                    (setq bal-ok false)
            })

            (if (> t-ic (bms-get-param 't_bal_max_ic)) {
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

                    (looprange i 0 cell-num (with-com `(bms-set-bal ,i ,(ix bal-chs i))))

                    (setq is-balancing (> ch-cnt 0))
            })

            (if (not bal-ok) {
                    (disable-balancing)
            })

            (setq bal-status (if is-balancing "BAL" ""))

            (var bal-ok-before bal-ok)
            (looprange i 0 15 {
                    (if (not (eq bal-ok-before bal-ok)) (break))
                    (sleep 1.0)
            })
}))

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
            (event-bms-zero-ofs (setq current-zero-offset (with-com '(bms-current-raw))))
            ((event-data-rx ? data) (handle-app-data data))
            (_ nil)
            ;((? a) (print a))
)))

(defun handle-app-data (data)
    (match (trap (read data))
        ((exit-ok (bms-shutdown)) {
            (print "APPUI requested BMS shutdown")
            (spawn (fn () (bms-shutdown-app)))
        })
        (_ (print "Ignoring unsupported APPUI command"))
))

(defun main () {
        (if (> app-wdt-timeout 0)
            (wdt-configure true app-wdt-timeout)
            (wdt-disable)
        )

        ; Compatibility Check
        (loopwhile (!= (bms-fw-version) 6) {
                (if (< (bms-fw-version) 6)
                    (print "Firmware too old, please update")
                    (print "Package too old, please update")
                )

                (gpio-write 9 0) ; Enable CAN
                (sleep 5)
        })

        (set-fw-name "")

        (def charge-dis-ts (systime))
        (def t-last (systime))
        (def charge-ts (systime))

        ; Buzzer
        (pwm-start 2730 0.0 0 3)

        (if (= (bufget-u8 (rtc-data) 900) rtc-val-magic) {
                (var tmp (unflatten (rtc-data)))
                (if tmp (setq rtc-val tmp))
        })
        (sync-bq-wake-debug-globals)

        (def active-cells-ic1 (bms-get-param 'cells_ic1))
        (def active-cells-ic2 (bms-get-param 'cells_ic2))
        (def active-temp-num (bms-get-param 'temp_num))
        (def active-temp-res (bms-get-param 'temp_res))
        (def active-max-charge-current (bms-get-param 'max_charge_current))
        (def cell-num (+ active-cells-ic1 active-cells-ic2))

        (def t-start-fun (secs-since 0))

        (loopwhile t {
                (match (trap (start-fun))
                    ((exit-ok (? a)) (break))
                    (_ nil)
                )
                (sleep 1.0)
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
        (def ah-cnt-soc (read-setting 'ah-cnt-soc))

        (event-register-handler (spawn event-handler))
        (event-enable 'event-bms-chg-allow)
        (event-enable 'event-bms-reset-cnt)
        (event-enable 'event-bms-force-bal)
        (event-enable 'event-bms-zero-ofs)
        (event-enable 'event-data-rx)

        (set-bms-val 'bms-cell-num cell-num)
        (set-bms-val 'bms-can-id (can-local-id))

        (loopwhile-thd ("main-ctrl" 200) t {
                (trap (main-ctrl))
                (setq did-crash true)
                (loopwhile did-crash (sleep 1.0))
        })

        (loopwhile-thd ("balance" 200) t {
                (trap (balance))
                (setq did-crash true)
                (loopwhile did-crash (sleep 1.0))
        })

        (loopwhile-thd ("re-init" 200) t {
                (var cfg-cells-ic1 (bms-get-param 'cells_ic1))
                (var cfg-cells-ic2 (bms-get-param 'cells_ic2))
                (var cfg-temp-num (bms-get-param 'temp_num))
                (var cfg-temp-res (bms-get-param 'temp_res))
                (var cfg-max-charge-current (bms-get-param 'max_charge_current))

                (if (or
                        (!= cfg-cells-ic1 active-cells-ic1)
                        (!= cfg-cells-ic2 active-cells-ic2)
                        (!= cfg-temp-num active-temp-num)
                        (!= cfg-temp-res active-temp-res)
                        (!= cfg-max-charge-current active-max-charge-current)
                    ) {
                        (print "BMS config changed, reinitializing hardware")
                        ; Fail closed before any BQ communication that could block.
                        (fail-close-outputs true)
                        (com-force true)
                        (init-hw)
                        (user-beep 2 0.05)
                        (setq active-cells-ic1 cfg-cells-ic1)
                        (setq active-cells-ic2 cfg-cells-ic2)
                        (setq active-temp-num cfg-temp-num)
                        (setq active-temp-res cfg-temp-res)
                        (setq active-max-charge-current cfg-max-charge-current)
                        (setq cell-num (+ active-cells-ic1 active-cells-ic2))
                        (set-bms-val 'bms-cell-num cell-num)
                        (set-bms-val 'bms-temp-adc-num (+ 5 (truncate active-temp-num 0 4)))
                        (com-force false)
                })

                (if did-crash {
                        ; Fail closed immediately. init-hw can loop while recovering
                        ; BQ communication, so do not leave the charge gate enabled
                        ; or balance channels active while recovery is waiting on the bus.
                        (fail-close-outputs true)
                        (com-force true)
                        (init-hw)
                        (com-force false)
                        (fail-close-outputs true)
                        (setq did-crash false)
                        (setq crash-cnt (+ crash-cnt 1))
                })

                (sleep 0.1)
        })

        (loopwhile-thd ("fail-close-retry" 100) t {
                (if fail-close-failed {
                        (fail-close-outputs true)
                })

                (if (and bal-off-failed (not fail-close-failed)) {
                        (disable-balancing)
                })

                (sleep 0.5)
        })

        (loopwhile-thd ("sleep-unblock" 100) t {
                (var sleep-unblock-ok (fn () (and
                            (= (bms-get-param 'block_sleep) 1)
                            (< (- c-max c-min) 0.05)
                            (> c-min 2.4)
                            (> (secs-since 0) 3600)
                            sleep-unblock-en
                )))

                (var should-unblock true)
                (looprange i 0 60 {
                        (if (not (sleep-unblock-ok)) {
                                (setq should-unblock false)
                        })
                        (sleep 1.0)
                })

                (if should-unblock {
                        (bms-set-param 'block_sleep 0)
                        (bms-store-cfg)
                        (print "Block sleep disabled")
                        (user-beep 4 0.2)

                })
        })
})

@const-end

(image-save)
(main)
