(import "font_16_26.bin" 'font)

(gpio-configure 0 'pin-mode-out)
(gpio-write 0 1)

(disp-load-st7789a 7 6 -1 8 10 80)
(disp-reset)
(ext-disp-orientation 1)

(touch-load-cst836u 4 5 2 -1 320 240 400000)
(touch-apply-transforms 0 1 1)

(def img (img-buffer 'indexed4 320 240))

(def gesture-label "No gesture")
(def touch-state 'idle) ; idle | down | move
(def press-start-x 0)
(def press-start-y 0)
(def press-start-t 0.0)
(def last-x 0)
(def last-y 0)

(def swipe-threshold 28)
(def tap-threshold 12)
(def long-press-threshold 10)

(defun abs-diff (a b)
    (if (> a b) (- a b) (- b a)))

(defun detect-gesture (x y ts pressed)
    (if pressed
        (if (eq touch-state 'idle)
            (progn
                (setq touch-state 'down)
                (setq press-start-x x)
                (setq press-start-y y)
                (setq press-start-t ts)
                (setq last-x x)
                (setq last-y y)
                (setq gesture-label "Touch"))
            (progn
                (setq touch-state 'move)
                (setq last-x x)
                (setq last-y y)
                (setq gesture-label "Moving")))
        (if (or (eq touch-state 'down) (eq touch-state 'move))
            (progn
                (var end-x last-x)
                (var end-y last-y)
                (var dx-abs (abs-diff end-x press-start-x))
                (var dy-abs (abs-diff end-y press-start-y))
                (var dt (secs-since press-start-t))
                (var moved-right (> end-x (+ press-start-x swipe-threshold)))
                (var moved-left (> press-start-x (+ end-x swipe-threshold)))
                (var moved-down (> end-y (+ press-start-y swipe-threshold)))
                (var moved-up (> press-start-y (+ end-y swipe-threshold)))

                (if (and (< dx-abs tap-threshold) (< dy-abs tap-threshold) (< dt 0.25))
                    (setq gesture-label "Tap")
                    (if (and (< dx-abs long-press-threshold) (< dy-abs long-press-threshold) (> dt 0.5))
                        (setq gesture-label "Long press")
                        (if (> dx-abs (+ dy-abs 6))
                            (if moved-right
                                (setq gesture-label "Swipe right")
                                (if moved-left
                                    (setq gesture-label "Swipe left")
                                    (setq gesture-label "Unknown gesture")))
                            (if (> dy-abs (+ dx-abs 6))
                                (if moved-down
                                    (setq gesture-label "Swipe down")
                                    (if moved-up
                                        (setq gesture-label "Swipe up")
                                        (setq gesture-label "Unknown gesture")))
                                (setq gesture-label "Unknown gesture")))))

                (setq touch-state 'idle))
            (setq gesture-label "No gesture"))))

(defun touch-event-handler ()
    (loopwhile t
        (recv
            ((event-touch-int cst836u (? pressed) (? x) (? y) (? strength) (? track-id))
                (detect-gesture x y (systime) pressed))
            (_ nil))))

(event-register-handler (spawn touch-event-handler))
(event-enable 'event-touch-int)

(loopwhile t {
    (img-clear img)

    (img-rectangle img 0 0 319 239 2)
    (img-text img 8 10 3 0 font "Gesture:")
    (img-text img 8 36 3 0 font gesture-label)

    (if (not (eq touch-state 'idle))
        (progn
            (img-circle img last-x last-y 16 3 '(thickness 2))
            (img-circle img last-x last-y 3 3 '(filled))))

    (disp-render img 0 0 (list 0x000000 0xffa000 0x00d0ff 0xffffff))
    (sleep 0.01)
})
