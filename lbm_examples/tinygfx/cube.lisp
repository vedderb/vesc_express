; Keep one display block uncommented. Adjust its GPIO pins for your board.
(import "font_16_26.bin" 'font)
(def render-y 0)

; ST7789: SD0=6 CLK=5 CS=19 RST=18 DC=7, 40 MHz
(disp-load-st7789 6 5 19 18 7 40)
(disp-reset)
(ext-disp-orientation 0)
(def size '(240 240))

; ST7735: SD0=6 CLK=5 CS=19 RST=18 DC=7, 40 MHz
; (disp-load-st7735 6 5 19 18 7 40)
; (disp-reset)
; (ext-disp-orientation 1)
; (def size '(160 80))
; (setq render-y 24)

; ILI9341: SD0=6 CLK=5 CS=19 RST=18 DC=7, 40 MHz
; (disp-load-ili9341 6 5 19 18 7 40)
; (disp-reset)
; (ext-disp-orientation 1)
; (def size '(320 240))

; ILI9488: SD0=6 CLK=5 CS=19 RST=18 DC=7, 40 MHz
; (disp-load-ili9488 6 5 19 18 7 40)
; (disp-reset)
; (ext-disp-orientation 0)
; (ext-disp-cmd 0x21) ; Inversion required by the original example panel.
; (def size '(320 480))

; SSD1351: SD0=6 CLK=5 CS=19 RST=18 DC=7, 40 MHz
; (disp-load-ssd1351 6 5 19 18 7 40)
; (disp-reset)
; (def size '(128 128))

; GC9A01: SD0=3 CLK=2 CS=5 RST=6 DC=4, 40 MHz
; (disp-load-gc9a01 3 2 5 6 4 40)
; (disp-reset)
; (ext-disp-orientation 0)
; (def size '(240 240))

; SH8601: SD0=13 CLK=10 CS=12 RST=9 DC=11, 40 MHz
; (disp-load-sh8601 13 10 12 9 11 40)
; (disp-reset)
; (ext-disp-orientation 1)
; (def size '(320 170))

; JD9853: SD0=2 CLK=1 CS=14 RST=22 DC=15 BL=23, 40 MHz
; (gpio-configure 23 'pin-mode-out)
; (gpio-write 23 0)
; (disp-load-jd9853 2 1 14 22 15 40)
; (disp-reset)
; (ext-disp-orientation 3)
; (gpio-write 23 1)
; (def size '(320 172))

; AXS15231: SD0=21 SD1=48 SD2=40 SD3=39 CLK=47 CS=45 RST=-1 BL=1, 80 MHz
; (gpio-configure 1 'pin-mode-out)
; (gpio-write 1 1)
; (disp-load-axs15231 21 48 40 39 47 45 -1 80)
; (disp-reset)
; (ext-disp-orientation 0)
; (def size '(320 480))

; ST7701 (ESP32P4, MIPI DSI): RST=5 BL=23, 1200 Mbps per lane
; (disp-load-st7701 5 1200)
; (ext-disp-orientation 0)
; (gpio-configure 23 'pin-mode-out)
; (gpio-write 23 1)
; (def size '(480 800))

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
    (img-text img 4 4 2 0 font (str-from-n fps "FPS %.1f"))
    (draw-cube)
    (rotate-cube 0.1 0.05)
    (disp-render img 0 render-y '(0x000000 0xffffff 0x00ff00 0xff0000))
    (setq fps (/ 1 (secs-since start)))
})
