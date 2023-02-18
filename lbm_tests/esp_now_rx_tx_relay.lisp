(esp-now-start)

(def proc-cnt 0)
(def send-cnt 0)
(def send-fail-cnt 0)

(defun proc-data (src des data)
    (progn
        (def proc-cnt (+ proc-cnt 1))
        (esp-now-add-peer src)
        (if (esp-now-send src data)
            (def send-cnt (+ send-cnt 1))
            (def send-fail-cnt (+ send-fail-cnt 1))
        )
))

(defun event-handler ()
    (loopwhile t
        (recv
            ((event-esp-now-rx (? src) (? des) (? data)) (proc-data src des data))
            (_ nil)
)))

(event-register-handler (spawn event-handler))
(event-enable 'event-esp-now-rx)
