(def lcd-rst 27)
(def lcd-bl 23)
(def touch-sda 7)
(def touch-scl 8)
(def touch-rst -1)
(def touch-int -1)
(def display-w 1024)
(def display-h 600)
(def touch-raw-w 800)
(def touch-raw-h 480)

(gpio-configure lcd-bl 'pin-mode-out)
(gpio-write lcd-bl 0)

(disp-load-jd9165 lcd-rst 750)
(disp-reset)
(ext-disp-orientation 0)

(touch-load-gt911 touch-sda touch-scl touch-rst touch-int touch-raw-w touch-raw-h 400000)
(touch-apply-transforms 0 0 0)

(gpio-write lcd-bl 1)

(def img (img-buffer 'rgb565 display-w display-h))
(def last-x 0)
(def last-y 0)
(def last-touch 0)

(defun clamp (v lo hi)
    (if (< v lo) lo (if (> v hi) hi v)))

; Scale raw touch (800x480) to display space (1024x600)
(defun touch-scale (x y) {
    (var sx (to-i (/ (* x display-w) touch-raw-w)))
    (var sy (to-i (/ (* y display-h) touch-raw-h)))
    (list
        (clamp sx 0 (- display-w 1))
        (clamp sy 0 (- display-h 1))
    )
})

(loopwhile t {
    (img-clear img 0x000000)
    (img-rectangle img 0 0 1023 599 0x00ffff)
    ;(img-rectangle img 6 6 1017 593 0x1d2530)
    (img-circle img 512 300 96 0x4ea3ff '(thickness 4))
    (img-circle img 512 300 8 0xffffff '(filled))

    (var tp (touch-read))
    (if tp {
        (var pxy (touch-scale (ix tp 0) (ix tp 1)))
        (setq last-x (ix pxy 0))
        (setq last-y (ix pxy 1))
        (setq last-touch (ix tp 2))
        (img-circle img last-x last-y 28 0xffc14d '(thickness 4))
        (img-circle img last-x last-y 6 0xffc14d '(filled))
    })

    (disp-render img 0 0)
    (sleep 0.01)
})
