(esp-now-start)

(defun proc-data (data)
    (print data)
)

(defun event-handler ()
    (loopwhile t
        (recv
            ((event-esp-now-rx . (? data)) (proc-data data))
            (_ nil)
)))

(event-register-handler (spawn event-handler))
(event-enable 'event-esp-now-rx)
