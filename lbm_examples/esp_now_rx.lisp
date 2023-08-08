(esp-now-start)

(defun proc-data (src des data rssi)
    (print (list "src:" src  "des:" des "data:" data "rssi:" rssi))
)
        
(defun event-handler ()
    (loopwhile t
        (recv
            ((event-esp-now-rx (? src) (? des) (? data) (? rssi)) (proc-data src des data rssi))
            (_ nil)
)))

(event-register-handler (spawn event-handler))
(event-enable 'event-esp-now-rx)
