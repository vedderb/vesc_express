
;(disp-load-axs15231  sd0 sd1 sd2 sd3 clk cs  rst mhz)
;(touch-load-axs15231 sda scl rst int w   h  [freq])

(import "font_16_26.bin" 'font)

(disp-load-axs15231 21 48 40 39 47 45 -1 40)
(disp-reset)
(touch-load-axs15231 4 8 -1 -1 320 480)

(gpio-configure 1 'pin-mode-out)
(gpio-write 1 1)

(ext-disp-orientation 0)

(def img (img-buffer 'indexed4 320 480))
;(def img (img-buffer 'indexed4 480 320))

(defun line (x0 y0 x1 y1)
    (img-line img x0 y0 x1 y1 1 '(thickness 2))
)

(def nodes '((-1 -1 -1) (-1 -1 1) (-1 1 -1) (-1 1 1) (1 -1 -1) (1 -1 1) (1 1 -1) (1 1 1)))
(def edges '((0 1) (1 3) (3 2) (2 0) (4 5) (5 7) (7 6) (6 4) (0 4) (1 5) (2 6) (3 7)))

(defun draw-edges () {
    (var scale 70.0)
    (var ofs-x (/ 160.0 scale))
    (var ofs-y (/ 240.0 scale))

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

(def TOUCH_X 0)
(def TOUCH_Y 0)

(touch-apply-transforms 0 0 0)

(defun print-touch-poll ()
    (let ((p (touch-read)))
        (if p
            (print (list 'poll-touch 'axs15231 p))
            nil)))

(defun touch-print-if-pressed (pressed x y strength track-id)
    (if pressed
        (progn
            (setq TOUCH_X x)
            (setq TOUCH_Y y)
            (print (list 'touch-int 'axs15231 TOUCH_X TOUCH_Y strength track-id)))
        nil))

(defun touch-event-handler ()
    (loopwhile t
        (recv
            ((event-touch-int axs15231 (? pressed) (? x) (? y) (? strength) (? track-id))
                (touch-print-if-pressed pressed x y strength track-id))
            (_ nil))))

(event-register-handler (spawn touch-event-handler))
(event-enable 'event-touch-int)

(def fps 0)

(loopwhile-thd 100 t {
    (var t-start (systime))

    ; FPS text
    (img-text img 10 10 3 0 font (str-from-n fps "FPS %.1f "))

    ; Border frame
    (img-rectangle img 0 0 319 479 2)
    ;(img-rectangle img 0 0 479 319 2)

    ; Draw rotating cube
    (draw-edges)
    (rotate-cube 0.05 0.03)

    ; Render to display (indexed4 with 4-color palette)
    (disp-render img 0 0 (list 0x000000 0xff4400 0x00ccff 0x22ff44))

    (img-clear img)
    (def fps (/ 1.0 (secs-since t-start)))
})
