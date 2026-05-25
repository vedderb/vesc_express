(import "font_16_26.bin" 'font)

(disp-load-st7701 5 1200)
(ext-disp-orientation 0)

(gpio-configure 23 'pin-mode-out)
(gpio-write 23 1)

(def img (img-buffer 'rgb565 480 800))

(defun line (x0 y0 x1 y1)
    (img-line img x0 y0 x1 y1 0xffffff )
)

(def nodes '((-1 -1 -1) (-1 -1 1) (-1 1 -1) (-1 1 1) (1 -1 -1) (1 -1 1) (1 1 -1) (1 1 1)))
(def edges '((0  1) (1 3) (3 2) (2 0) (4 5) (5 7) (7 6) (6 4) (0 4) (1 5) (2 6) (3 7)))

(defun draw-edges () {
        (var scale 120.0)
        (var ofs-x (/ 240 scale))
        (var ofs-y (/ 400 scale))

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
        (img-text img 10 10 0xffffff 0x000000 font (str-from-n fps "FPS %.1f"))
        (img-circle img 240 400 220 0xff0000 '(thickness 5))
        (img-rectangle img 0 1 479 798 0x00ff00)
        (draw-edges)
        (rotate-cube 0.1 0.05)
        (disp-render img 0 0)
        (img-clear img 0)
        (def fps (/ 1 (secs-since t-start)))
})