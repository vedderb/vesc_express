(def touch-sda 47)
(def touch-scl 48)
(def touch-rst 17)
(def touch-int 21)
(def touch-width 320)
(def touch-height 170)

(def TOUCH_X 0)
(def TOUCH_Y 0)

(touch-load-cst816s touch-sda touch-scl touch-rst touch-int touch-width touch-height 400000)

(touch-swap-xy 0)
(touch-mirror-x 0)
(touch-mirror-y 0)

(defun print-touch-poll ()
    (let ((p (touch-read-and-get)))
        (if p
            (print (list 'poll-touch p))
            nil)))

(defun touch-event-handler ()
    (loopwhile t
        (recv
            ((event-touch-int cst816s (? pressed) (? x) (? y) (? strength) (? track-id))
                (if pressed
                    (progn
                        (setq TOUCH_X x)
                        (setq TOUCH_Y y)
                        (print (list 'touch-int TOUCH_X TOUCH_Y strength track-id)))
                    nil))
            (_ nil))))

(event-register-handler (spawn touch-event-handler))
(event-enable 'event-touch-int)