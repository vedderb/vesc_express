(def touch-width 320)
(def touch-height 170)

; CST816S pins
(def touch-sda 47)
(def touch-scl 48)
(def touch-rst 17)
(def touch-int 21)
(def touch-i2c-freq 400000)

(def TOUCH_X 0)
(def TOUCH_Y 0)

(touch-load-cst816s touch-sda touch-scl touch-rst touch-int touch-width touch-height touch-i2c-freq)

(touch-apply-transforms 0 0 0)

(defun print-touch-poll ()
    (let ((p (touch-read)))
        (if p
            (print (list 'poll-touch 'cst816s p))
            nil)))

(defun touch-print-if-pressed (pressed x y strength track-id)
    (if pressed
        (progn
            (setq TOUCH_X x)
            (setq TOUCH_Y y)
            (print (list 'touch-int 'cst816s TOUCH_X TOUCH_Y strength track-id)))
        nil))

(defun touch-event-handler ()
    (loopwhile t
        (recv
            ((event-touch-int cst816s (? pressed) (? x) (? y) (? strength) (? track-id))
                (touch-print-if-pressed pressed x y strength track-id))
            (_ nil))))

(event-register-handler (spawn touch-event-handler))
(event-enable 'event-touch-int)