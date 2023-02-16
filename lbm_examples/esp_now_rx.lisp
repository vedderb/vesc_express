(esp-now-start)

(defun proc-data (src des data)
    (print (list "src:" src  "des:" des "data:" data))
)
        
(defun event-handler ()
    (loopwhile t
        (recv
            ((event-esp-now-rx (? src) (? des) (? data)) (proc-data src des data))
            (_ nil)
)))

(event-register-handler (spawn event-handler))
(event-enable 'event-esp-now-rx)
