; Speaker input low to prevent idle noise.
(gpio-configure 25 'pin-mode-out)
(gpio-write 25 0)

(def render-y 0)

; The LCD and SD card share SPI. Keep the card deselected.
(gpio-configure 4 'pin-mode-out)
(gpio-write 4 1)
(gpio-configure 32 'pin-mode-out)
(gpio-write 32 0)

(disp-load-ili9341 23 18 14 33 27 40)
(disp-reset)

; ILI9342C power, gamma and native landscape geometry.
(ext-disp-cmd 0xC8 0xFF 0x93 0x42)
(ext-disp-cmd 0xC0 0x12 0x12)
(ext-disp-cmd 0xC1 0x03)
(ext-disp-cmd 0xC5 0xF2)
(ext-disp-cmd 0xB0 0xE0)
(ext-disp-cmd 0xF6 0x01 0x00 0x00)
(ext-disp-cmd 0xE0 0 12 17 4 17 8 55 137 76 6 12 10 46 52 15)
(ext-disp-cmd 0xE1 0 11 17 5 19 9 51 103 72 7 14 11 46 51 15)
(ext-disp-cmd 0xB6 8 130 29 4)
(ext-disp-cmd 0x36 0x08)
(ext-disp-cmd 0x3A 0x55)

;(ext-disp-cmd 0x20) ; M5Stack Basic 2018.3 has a TN panel, so invert the display.
(ext-disp-cmd 0x21) ; M5Stack Basic v2.7 has an IPS panel, so do not invert the display.

(disp-clear 0x204080)
(gpio-write 32 1)

(def size '(320 240))

(def width (ix size 0))
(def height (ix size 1))
(def scale (* (if (< width height) width height) 0.22))
(def img (img-buffer 'indexed4 width height))
(def nodes '((-1 -1 -1) (-1 -1 1) (-1 1 -1) (-1 1 1)
             (1 -1 -1) (1 -1 1) (1 1 -1) (1 1 1)))
(def edges '((0 1) (1 3) (3 2) (2 0) (4 5) (5 7)
             (7 6) (6 4) (0 4) (1 5) (2 6) (3 7)))

(defun draw-cube () {
    (loopforeach e edges {
        (var a (ix nodes (ix e 0)))
        (var b (ix nodes (ix e 1)))
        (img-line img
            (to-i (+ (/ width 2) (* (ix a 0) scale)))
            (to-i (+ (/ height 2) (* (ix a 1) scale)))
            (to-i (+ (/ width 2) (* (ix b 0) scale)))
            (to-i (+ (/ height 2) (* (ix b 1) scale)))
            1 '(thickness 2))
    })
})

(defun rotate-cube (ax ay) {
    (var sx (sin ax)) (var cx (cos ax))
    (var sy (sin ay)) (var cy (cos ay))
    (loopforeach n nodes {
        (var x (ix n 0)) (var y (ix n 1)) (var z (ix n 2))
        (setix n 0 (- (* x cx) (* z sx)))
        (setix n 2 (+ (* z cx) (* x sx)))
        (setq z (ix n 2))
        (setix n 1 (- (* y cy) (* z sy)))
        (setix n 2 (+ (* z cy) (* y sy)))
    })
})

(def fps 0)
(loopwhile t {
    (var start (systime))
    (img-clear img)
    (img-text img 4 4 2 0 (str-from-n fps "FPS %.1f"))
    (draw-cube)
    (rotate-cube 0.1 0.05)
    (disp-render img 0 render-y '(0x000000 0xffffff 0x00ff00 0xff0000))
    (setq fps (/ 1 (secs-since start)))
})
