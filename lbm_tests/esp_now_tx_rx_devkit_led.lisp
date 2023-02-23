(esp-now-start)

(def other-peer '(112 4 29 15 194 105))

(esp-now-add-peer other-peer)

(defun proc-data (src des data)
    (eval (read data))
)
        
(defun event-handler ()
    (loopwhile t
        (recv
            ((event-esp-now-rx (? src) (? des) (? data)) (proc-data src des data))
            (_ nil)
)))

(event-register-handler (spawn event-handler))
(event-enable 'event-esp-now-rx)

(rgbled-init 8 1)

(loopwhile t
    (progn
        (if (not (esp-now-send other-peer "(rgbled-color 0 0x220000)")) (rgbled-color 0))
        (def state 1) (sleep 0.005)
        (if (not (esp-now-send other-peer "(rgbled-color 0 0x002200)")) (rgbled-color 0))
        (def state 2) (sleep 0.005)
        (if (not (esp-now-send other-peer "(rgbled-color 0 0x000022)")) (rgbled-color 0))
        (def state 3) (sleep 0.005)
))