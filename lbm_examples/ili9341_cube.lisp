
(import "font_16_26.bin" 'font)

(disp-load-ili9341 6 5 19 18 7 40)
(disp-reset)

(def img (img-buffer 'indexed2 320 240))

(defun line (x0 y0 x1 y1)
    (img-line img x0 y0 x1 y1 1 '(thickness 2))
)

; Nodes and edges of a 3d cube
(def nodes '((-1 -1 -1) (-1 -1 1) (-1 1 -1) (-1 1 1) (1 -1 -1) (1 -1 1) (1 1 -1) (1 1 1)))
(def edges '((0  1) (1 3) (3 2) (2 0) (4 5) (5 7) (7 6) (6 4) (0 4) (1 5) (2 6) (3 7)))

(defun draw-edges ()
    (let (
        (scale 60.0)
        (ofs-x (/ 160 scale))
        (ofs-y (/ 120 scale)))
        (loopforeach e edges
            (let (
                (na (ix nodes (ix e 0)))
                (nb (ix nodes (ix e 1))))
                (apply line (map (fn (x) (to-i (* x scale))) (list
                    (+ ofs-x (ix na 0)) (+ ofs-y (ix na 1))
                    (+ ofs-x (ix nb 0)) (+ ofs-y (ix nb 1))
)))))))

(defun rotate (ax ay) (let (
    (sx (sin ax))
    (cx (cos ax))
    (sy (sin ay))
    (cy (cos ay)))
    (loopforeach n nodes
        (let (
            (x (ix n 0))
            (y (ix n 1))
            (z (ix n 2)))
            (progn
                (setix n 0 (- (* x cx) (* z sx)))
                (setix n 2 (+ (* z cx) (* x sx)))
                (setvar 'z (ix n 2))
                (setix n 1 (- (* y cy) (* z sy)))
                (setix n 2 (+ (* z cy) (* y sy)))
)))))

(def fps 0)

(loopwhile t
    (progn
        (def t-start (systime))
        (img-text img 4 210 1 0 font (str-from-n fps "FPS %.1f "))
        (draw-edges)
        (rotate 0.1 0.05)
        (disp-render img 0 0 '(0 0x00ff00))
        (img-clear img)
        (def fps (/ 1 (secs-since t-start)))
))