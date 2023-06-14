(bms-init)

(defun go-to-sleep () {
        (bms-sleep)
        (bms-set-btn-wakeup-state 1)
        (sleep-deep -1)
})

(loopwhile t {
        (def vtot (apply + (bms-get-vcells)))
        (def vout (bms-get-vout))
        (def vchg (bms-get-vchg))
        (def iout (bms-get-current))
        (def btn (bms-get-btn))
        (def t1 (ix (bms-get-temps) 0))
        (sleep 0.1)
})
