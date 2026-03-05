(import "font_16_26.bin" 'font)

;(disp-load-sh8601 6 5 19 18 7 40)
(disp-load-sh8601 13 10 12 9 11 40)
(disp-reset)

; SH8601 initialization sequence from vendor's datasheet 
(defun sh8601-init () {
    (ext-disp-cmd 0x36 0x00)
    (ext-disp-cmd 0x3A 0x55)
    (ext-disp-cmd 0xB2 0x0C 0x0C 0x00 0x33 0x33)
    (ext-disp-cmd 0xB7 0x35)
    (ext-disp-cmd 0xBB 0x13)
    (ext-disp-cmd 0xC0 0x2C)
    (ext-disp-cmd 0xC2 0x01)
    (ext-disp-cmd 0xC3 0x0B)
    (ext-disp-cmd 0xC4 0x20)
    (ext-disp-cmd 0xC6 0x0F)
    (ext-disp-cmd 0xD0 0xA4 0xA1)
    (ext-disp-cmd 0xD6 0xA1)
    (ext-disp-cmd 0xE0 0x00 0x03 0x07 0x08 0x07 0x15 0x2A 0x44 0x42 0x0A 0x17 0x18 0x25 0x27)
    (ext-disp-cmd 0xE1 0x00 0x03 0x08 0x07 0x07 0x23 0x2A 0x43 0x42 0x09 0x18 0x17 0x25 0x27)
    (ext-disp-cmd 0x11)
    (sleep 0.12)
    (ext-disp-cmd 0x21)
    (sleep 0.02)
    (ext-disp-cmd 0x29)
    (sleep 0.12)
    (disp-clear 0x000000)
})

(sh8601-init)

(def img (img-buffer 'indexed2 160 80))

(defun line (x0 y0 x1 y1)
    (img-line img x0 y0 x1 y1 1 '(thickness 2))
)

; Nodes and edges of a 3d cube
(def nodes '((-1 -1 -1) (-1 -1 1) (-1 1 -1) (-1 1 1) (1 -1 -1) (1 -1 1) (1 1 -1) (1 1 1)))
(def edges '((0  1) (1 3) (3 2) (2 0) (4 5) (5 7) (7 6) (6 4) (0 4) (1 5) (2 6) (3 7)))

(defun draw-edges () {
        (var scale 20.0)
        (var ofs-x (/ 40 scale))
        (var ofs-y (/ 40 scale))

        (loopforeach e edges {
                (var na (ix nodes (ix e 0)))
                (var nb (ix nodes (ix e 1)))

                (apply line (map (fn (x) (to-i (* x scale))) (list
                            (+ ofs-x (ix na 0)) (+ ofs-y (ix na 1))
                            (+ ofs-x (ix nb 0)) (+ ofs-y (ix nb 1))
                )))
        })
})

(defun rotate-cube (ax ay) {
        (var sx (sin ax))
        (var cx (cos ax))
        (var sy (sin ay))
        (var cy (cos ay))

        (loopforeach n nodes {
                (var x (ix n 0))
                (var y (ix n 1))
                (var z (ix n 2))

                (setix n 0 (- (* x cx) (* z sx)))
                (setix n 2 (+ (* z cx) (* x sx)))
                (setq z (ix n 2))
                (setix n 1 (- (* y cy) (* z sy)))
                (setix n 2 (+ (* z cy) (* y sy)))
        })
})

(def fps 0)

(loopwhile t {
        (var t-start (systime))
        (img-text img 90 10 1 0 font "FPS")
        (img-text img 90 40 1 0 font (str-from-n fps "%.1f "))
        (draw-edges)
        (rotate-cube 0.1 0.05)
        (disp-render img 0 24 '(0 0xff0000))
        (img-clear img)
        (def fps (/ 1 (secs-since t-start)))
})
