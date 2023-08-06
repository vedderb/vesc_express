(bms-init)
(bms-set-pchg 0)
(bms-set-out 0)

; Enable charging
(bms-set-chg 1)

(def psw-state false)

(defun go-to-sleep () {
        (bms-sleep)
        (bms-set-btn-wakeup-state 1)
        (sleep-deep -1)
})

(defun psw-on () {
        (var res nil)
        (var v-tot (apply + (bms-get-vcells)))
        
        (bms-set-pchg 0)
        (bms-set-out 0)
        
        (var t-start (systime))
        (bms-set-pchg 1)
        
        (loopwhile (< (secs-since t-start) 0.2) {
                (if (> (bms-get-vout) (* v-tot 0.85)) {
                        (setq res t)
;                        (print (str-from-n (* (secs-since t-start) 1000.0) "PCHG T: %.1f ms"))
                        (break)
                })
                (sleep 0.01)
        })
        
        (if res
            (bms-set-out 1)
            {
                (bms-set-pchg 0)
;                (print "Timed out, make sure that there is no short on the output!")
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

; PSW Control
(spawn 100 (fn () (loopwhile t {
                (if (and (= (bms-get-btn) 0) psw-state) {
                        (psw-off)
                })
                
                (if (and (= (bms-get-btn) 1) (not psw-state)) {
                        (psw-on)
                })
                
                (sleep 0.1)
})))

(set-bms-val 'bms-cell-num 13)
(set-bms-val 'bms-temp-adc-num 3)

(loopwhile t {
        (var v-cells (bms-get-vcells))
        (def vtot (apply + v-cells))
        (def vout (bms-get-vout))
        (def vchg (bms-get-vchg))
        (def iout (bms-get-current))
        (def btn (bms-get-btn))
        (def t1 (ix (bms-get-temps) 0))
        (def t2 (ix (bms-get-temps) 1))
        (def t3 (ix (bms-get-temps) 2))
        
        (var ind 0)
        (loopforeach v v-cells {
                (set-bms-val 'bms-v-cell ind v)
                (setq ind (+ ind 1))
        })
        
        (set-bms-val 'bms-v-tot vtot)
        (set-bms-val 'bms-v-charge vchg)
        (set-bms-val 'bms-i-in-ic iout)
        (set-bms-val 'bms-temps-adc 0 t1)
        (set-bms-val 'bms-temps-adc 1 t2)
        (set-bms-val 'bms-temps-adc 2 t3)
        (set-bms-val 'bms-temp-cell-max t1)
        
        (send-bms-can)
        (sleep 0.1)
})
