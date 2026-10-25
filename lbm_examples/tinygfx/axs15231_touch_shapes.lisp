(import "font_16_26.bin" 'font)

(disp-load-axs15231 21 48 40 39 47 45 -1 40)
(disp-reset)
(ext-disp-orientation 0)

(gpio-configure 1 'pin-mode-out)
(gpio-write 1 1)

(def touch-sda 4)
(def touch-scl 8)
(def touch-rst -1)
(def touch-int -1)
(def DISP_W 320)
(def DISP_H 480)
(def HUD_TOP_H 36)
(def HUD_BOTTOM_H 28)
(def SCENE_CX (/ DISP_W 2))
(def SCENE_CY (/ (+ HUD_TOP_H (- DISP_H HUD_BOTTOM_H)) 2))

(touch-load-axs15231 touch-sda touch-scl touch-rst touch-int DISP_W DISP_H 400000)
(touch-apply-transforms 0 0 0)

(def img (img-buffer 'rgb565 DISP_W DISP_H))

(def COLOR_BG    0x050810)
(def COLOR_TEXT  0xe8f4ff)
(def COLOR_FRAME 0x162030)
(def COLOR_HUD   0x203050)
(def COLOR_TOUCH 0xffffff)
(def PAGE_COLORS '(0x4dd9ff 0xff8c42 0x6bff95 0xff6bc6))
(def PAGE_NAMES  '("HYPER4D" "OCTA3D" "ICOSA3D" "STAR3D"))

(def current-page 0)
(def page-count 4)
(def button-w 64)
(def button-h 36)
(def button-margin 10)
(def button-top (+ HUD_TOP_H 10))
(def prev-button-x button-margin)
(def next-button-x (- DISP_W button-margin button-w))
(def nav-button-y button-top)
(def button-debounce 180)
(def last-button-time 0)
(def touch-x 0)
(def touch-y 0)
(def touch-down false)
(def touch-latched false)

(def fps 0.0)

(def hc-nodes '(
    (-1 -1 -1 -1 0xff6b6b)
    ( 1 -1 -1 -1 0xff9f6b)
    (-1  1 -1 -1 0xffe36b)
    ( 1  1 -1 -1 0xd4ff6b)
    (-1 -1  1 -1 0x6bff95)
    ( 1 -1  1 -1 0x6bffd7)
    (-1  1  1 -1 0x6bcfff)
    ( 1  1  1 -1 0x6b8cff)
    (-1 -1 -1  1 0x8d6bff)
    ( 1 -1 -1  1 0xc66bff)
    (-1  1 -1  1 0xff6bf2)
    ( 1  1 -1  1 0xff6bb0)
    (-1 -1  1  1 0xffffff)
    ( 1 -1  1  1 0xffc7a0)
    (-1  1  1  1 0xa0ffd8)
    ( 1  1  1  1 0xa0c7ff)
))

(def hc-edges '(
    (0 1 0) (2 3 0) (4 5 0) (6 7 0) (8 9 0) (10 11 0) (12 13 0) (14 15 0)
    (0 2 1) (1 3 1) (4 6 1) (5 7 1) (8 10 1) (9 11 1) (12 14 1) (13 15 1)
    (0 4 2) (1 5 2) (2 6 2) (3 7 2) (8 12 2) (9 13 2) (10 14 2) (11 15 2)
    (0 8 3) (1 9 3) (2 10 3) (3 11 3) (4 12 3) (5 13 3) (6 14 3) (7 15 3)
))

(def HC_EDGE_COLORS '(0xff5a36 0x4dd9ff 0x9dff5a 0xffd84d))
(def HC_SCALE 700.0)
(def HC_CX SCENE_CX)
(def HC_CY SCENE_CY)

(defun hc-project (n) {
    (var x (ix n 0))
    (var y (ix n 1))
    (var z (ix n 2))
    (var w (ix n 3))
    (var wp (/ 1.0 (- 3.3 w)))
    (var x3 (* x wp))
    (var y3 (* y wp))
    (var z3 (* z wp))
    (var zp (/ 1.0 (- 4.0 z3)))
    (list (+ HC_CX (* x3 zp HC_SCALE))
          (+ HC_CY (* y3 zp HC_SCALE))
          (* wp zp))
})

(defun hc-draw-edge (e) {
    (var pa (hc-project (ix hc-nodes (ix e 0))))
    (var pb (hc-project (ix hc-nodes (ix e 1))))
    (var color (ix HC_EDGE_COLORS (ix e 2)))
    (img-line img
        (to-i (ix pa 0)) (to-i (ix pa 1))
        (to-i (ix pb 0)) (to-i (ix pb 1))
        color '(thickness 2))
})

(defun hc-draw-node (n) {
    (var p (hc-project n))
    (var r (+ 3 (to-i (* 8.0 (ix p 2)))))
    (img-circle img (to-i (ix p 0)) (to-i (ix p 1)) r (ix n 4) '(filled))
})

(defun hc-rotate (a b c d) {
    (var sa (sin a)) (var ca (cos a))
    (var sb (sin b)) (var cb (cos b))
    (var sc (sin c)) (var cc (cos c))
    (var sd (sin d)) (var cd (cos d))
    (loopforeach n hc-nodes {
        (var x (ix n 0)) (var y (ix n 1))
        (var z (ix n 2)) (var w (ix n 3))
        (setix n 0 (- (* x ca) (* w sa)))
        (setix n 3 (+ (* w ca) (* x sa)))
        (setq x (ix n 0)) (setq w (ix n 3))
        (setix n 1 (- (* y cb) (* z sb)))
        (setix n 2 (+ (* z cb) (* y sb)))
        (setq y (ix n 1)) (setq z (ix n 2))
        (setix n 2 (- (* z cc) (* w sc)))
        (setix n 3 (+ (* w cc) (* z sc)))
        (setq z (ix n 2))
        (setix n 0 (- (* x cd) (* y sd)))
        (setix n 1 (+ (* y cd) (* x sd)))
    })
})

(defun draw-page-hypercube () {
    (loopforeach e hc-edges { (hc-draw-edge e) })
    (loopforeach n hc-nodes  { (hc-draw-node n) })
    (hc-rotate 0.041 0.027 0.033 0.018)
})

(def oct-nodes '(
    ( 0  0  1 0xffd84d)
    ( 0  0 -1 0x4dd9ff)
    ( 1  0  0 0xff6b6b)
    (-1  0  0 0x6bff95)
    ( 0  1  0 0xc66bff)
    ( 0 -1  0 0xff9f6b)
))

(def oct-edges '(
    (0 2) (0 3) (0 4) (0 5)
    (1 2) (1 3) (1 4) (1 5)
    (2 4) (4 3) (3 5) (5 2)
))

(def OCT_SCALE 280.0)
(def OCT_CX SCENE_CX)
(def OCT_CY SCENE_CY)

(defun oct-project (n) {
    (var x (ix n 0))
    (var y (ix n 1))
    (var z (ix n 2))
    (var zp (/ 1.0 (- 3.5 z)))
    (list (+ OCT_CX (* x zp OCT_SCALE))
          (+ OCT_CY (* y zp OCT_SCALE))
          zp)
})

(defun oct-draw () {
    (loopforeach e oct-edges {
        (var na (ix oct-nodes (ix e 0)))
        (var nb (ix oct-nodes (ix e 1)))
        (var pa (oct-project na))
        (var pb (oct-project nb))
        (var ca (ix na 3))
        (img-line img
            (to-i (ix pa 0)) (to-i (ix pa 1))
            (to-i (ix pb 0)) (to-i (ix pb 1))
            ca '(thickness 2))
    })
    (loopforeach n oct-nodes {
        (var p (oct-project n))
        (var r (+ 5 (to-i (* 6.0 (ix p 2)))))
        (img-circle img (to-i (ix p 0)) (to-i (ix p 1)) r (ix n 3) '(filled))
    })
})

(defun oct-rotate (ax ay) {
    (var sx (sin ax)) (var cx (cos ax))
    (var sy (sin ay)) (var cy (cos ay))
    (loopforeach n oct-nodes {
        (var x (ix n 0)) (var y (ix n 1)) (var z (ix n 2))
        (setix n 0 (- (* x cx) (* z sx)))
        (setix n 2 (+ (* z cx) (* x sx)))
        (setq z (ix n 2))
        (setix n 1 (- (* y cy) (* z sy)))
        (setix n 2 (+ (* z cy) (* y sy)))
    })
})

(defun draw-page-octahedron () {
    (oct-draw)
    (oct-rotate 0.035 0.051)
})

(def phi 1.618034)

(def ico-nodes (list
    (list  0.0   1.0  phi  0xff6b6b)
    (list  0.0  -1.0  phi  0xff9f6b)
    (list  0.0   1.0 (- phi)  0xffe36b)
    (list  0.0  -1.0 (- phi)  0xd4ff6b)
    (list  1.0   phi  0.0  0x6bff95)
    (list -1.0   phi  0.0  0x6bffd7)
    (list  1.0  (- phi)  0.0  0x6bcfff)
    (list -1.0  (- phi)  0.0  0x6b8cff)
    (list  phi   0.0   1.0  0xc66bff)
    (list (- phi)  0.0   1.0  0xff6bf2)
    (list  phi   0.0  -1.0  0xff6bb0)
    (list (- phi)  0.0  -1.0  0xffffff)
))

(def ico-edges '(
    (0 1) (0 4) (0 5) (0 8) (0 9)
    (1 6) (1 7) (1 8) (1 9)
    (2 3) (2 4) (2 5) (2 10) (2 11)
    (3 6) (3 7) (3 10) (3 11)
    (4 5) (4 8) (4 10)
    (5 9) (5 11)
    (6 7) (6 8) (6 10)
    (7 9) (7 11)
    (8 10) (9 11)
))

(def ICO_SCALE 205.0)
(def ICO_CX SCENE_CX)
(def ICO_CY SCENE_CY)

(defun ico-project (n) {
    (var x (ix n 0))
    (var y (ix n 1))
    (var z (ix n 2))
    (var zp (/ 1.0 (- 4.0 z)))
    (list (+ ICO_CX (* x zp ICO_SCALE))
          (+ ICO_CY (* y zp ICO_SCALE))
          zp)
})

(defun ico-draw () {
    (loopforeach e ico-edges {
        (var na (ix ico-nodes (ix e 0)))
        (var nb (ix ico-nodes (ix e 1)))
        (var pa (ico-project na))
        (var pb (ico-project nb))
        (img-line img
            (to-i (ix pa 0)) (to-i (ix pa 1))
            (to-i (ix pb 0)) (to-i (ix pb 1))
            (ix na 3) '(thickness 2))
    })
    (loopforeach n ico-nodes {
        (var p (ico-project n))
        (var r (+ 4 (to-i (* 5.0 (ix p 2)))))
        (img-circle img (to-i (ix p 0)) (to-i (ix p 1)) r (ix n 3) '(filled))
    })
})

(defun ico-rotate (ax ay az) {
    (var sx (sin ax)) (var cx (cos ax))
    (var sy (sin ay)) (var cy (cos ay))
    (var sz (sin az)) (var cz (cos az))
    (loopforeach n ico-nodes {
        (var x (ix n 0)) (var y (ix n 1)) (var z (ix n 2))
        (setix n 1 (- (* y cx) (* z sx)))
        (setix n 2 (+ (* z cx) (* y sx)))
        (setq y (ix n 1)) (setq z (ix n 2))
        (setix n 0 (- (* x cy) (* z sy)))
        (setix n 2 (+ (* z cy) (* x sy)))
        (setq x (ix n 0)) (setq z (ix n 2))
        (setix n 0 (- (* x cz) (* y sz)))
        (setix n 1 (+ (* y cz) (* x sz)))
    })
})

(defun draw-page-icosahedron () {
    (ico-draw)
    (ico-rotate 0.029 0.037 0.019)
})

(def star-nodes (list
    (list  1.0   1.0   1.0  0xff5a36)
    (list -1.0  -1.0   1.0  0xffd84d)
    (list -1.0   1.0  -1.0  0xff9f6b)
    (list  1.0  -1.0  -1.0  0xffe36b)
    (list -1.0  -1.0  -1.0  0x4dd9ff)
    (list  1.0   1.0  -1.0  0x6bff95)
    (list  1.0  -1.0   1.0  0x6bcfff)
    (list -1.0   1.0   1.0  0xc66bff)
))

(def star-edges '(
    (0 1) (0 2) (0 3)
    (1 2) (1 3) (2 3)
    (4 5) (4 6) (4 7)
    (5 6) (5 7) (6 7)
))

(def STAR_SCALE 200.0)
(def STAR_CX SCENE_CX)
(def STAR_CY SCENE_CY)

(defun star-project (n) {
    (var x (ix n 0))
    (var y (ix n 1))
    (var z (ix n 2))
    (var zp (/ 1.0 (- 4.0 z)))
    (list (+ STAR_CX (* x zp STAR_SCALE))
          (+ STAR_CY (* y zp STAR_SCALE))
          zp)
})

(defun star-draw () {
    (loopforeach e star-edges {
        (var na (ix star-nodes (ix e 0)))
        (var nb (ix star-nodes (ix e 1)))
        (var pa (star-project na))
        (var pb (star-project nb))
        (img-line img
            (to-i (ix pa 0)) (to-i (ix pa 1))
            (to-i (ix pb 0)) (to-i (ix pb 1))
            (ix na 3) '(thickness 3))
    })
    (loopforeach n star-nodes {
        (var p (star-project n))
        (var r (+ 5 (to-i (* 7.0 (ix p 2)))))
        (img-circle img (to-i (ix p 0)) (to-i (ix p 1)) r (ix n 3) '(filled))
    })
})

(defun star-rotate (ax ay) {
    (var sx (sin ax)) (var cx (cos ax))
    (var sy (sin ay)) (var cy (cos ay))
    (loopforeach n star-nodes {
        (var x (ix n 0)) (var y (ix n 1)) (var z (ix n 2))
        (setix n 0 (- (* x cx) (* z sx)))
        (setix n 2 (+ (* z cx) (* x sx)))
        (setq z (ix n 2))
        (setix n 1 (- (* y cy) (* z sy)))
        (setix n 2 (+ (* z cy) (* y sy)))
    })
})

(defun draw-page-star () {
    (star-draw)
    (star-rotate 0.047 0.031)
})

(defun draw-page-indicators () {
    (var dot-r 5)
    (var dot-gap 25)
    (var total-w (* (- page-count 1) dot-gap))
    (var start-x (- (/ DISP_W 2) (/ total-w 2)))
    (var dot-y (- DISP_H 14))
    (looprange i 0 page-count {
        (var cx (+ start-x (* i dot-gap)))
        (if (= i current-page)
            (img-circle img cx dot-y dot-r (ix PAGE_COLORS i) '(filled))
            (img-circle img cx dot-y dot-r (ix PAGE_COLORS i)))
    })
})

(defun draw-hud () {
    (var page-color (ix PAGE_COLORS current-page))
    (var page-name  (ix PAGE_NAMES  current-page))
    (img-rectangle img 0 0 DISP_W HUD_TOP_H COLOR_HUD '(filled))
    (img-line img 0 HUD_TOP_H DISP_W HUD_TOP_H page-color '(thickness 1))
    (img-text img 8 7 page-color COLOR_HUD font page-name)
    (img-text img 210 7 COLOR_TEXT COLOR_HUD font
              (str-from-n fps "%.0f FPS"))
    (img-rectangle img 0 (- DISP_H HUD_BOTTOM_H) DISP_W HUD_BOTTOM_H COLOR_HUD '(filled))
    (img-line img 0 (- DISP_H HUD_BOTTOM_H) DISP_W (- DISP_H HUD_BOTTOM_H) page-color '(thickness 1))
    (draw-page-indicators)
    (draw-nav-buttons)
})

(defun point-in-rect (px py rx ry rw rh) {
    (and (>= px rx)
         (< px (+ rx rw))
         (>= py ry)
         (< py (+ ry rh)))
})

(defun nav-prev-enabled () {
    (> current-page 0)
})

(defun nav-next-enabled () {
    (< current-page (- page-count 1))
})

(defun nav-prev-pressed () {
    (and touch-down
         (point-in-rect touch-x touch-y prev-button-x nav-button-y button-w button-h))
})

(defun nav-next-pressed () {
    (and touch-down
         (point-in-rect touch-x touch-y next-button-x nav-button-y button-w button-h))
})

(defun draw-nav-button (x y w h label enabled pressed accent) {
    (var fill COLOR_FRAME)
    (var text-color 0x5c6878)
    (if enabled
        (progn
            (setq fill COLOR_HUD)
            (setq text-color COLOR_TEXT))
        nil)
    (if pressed
        (progn
            (setq fill accent)
            (setq text-color COLOR_BG))
        nil)
    (img-rectangle img x y w h fill '(filled))
    (img-rectangle img x y w h accent)
    (img-text img (+ x 22) (+ y 6) text-color fill font label)
})

(defun draw-nav-buttons () {
    (var page-color (ix PAGE_COLORS current-page))
    (draw-nav-button prev-button-x nav-button-y button-w button-h "<" (nav-prev-enabled) (and (nav-prev-enabled) (nav-prev-pressed)) page-color)
    (draw-nav-button next-button-x nav-button-y button-w button-h ">" (nav-next-enabled) (and (nav-next-enabled) (nav-next-pressed)) page-color)
})

(defun handle-buttons () {
    (var touch-data (touch-read))
    (var now (systime))
    (var elapsed (- now last-button-time))
    (if touch-data
        (progn
            (setq touch-down true)
            (setq touch-x (ix touch-data 0))
            (setq touch-y (ix touch-data 1)))
        (progn
            (setq touch-down false)
            (setq touch-latched false)))

    (if (and touch-down (not touch-latched) (> elapsed button-debounce))
        (progn
            (if (and (nav-prev-enabled) (nav-prev-pressed))
                (progn
                    (setq current-page (- current-page 1))
                    (setq last-button-time now)
                    (setq touch-latched true))
                nil)
            (if (and (nav-next-enabled) (nav-next-pressed))
                (progn
                    (setq current-page (+ current-page 1))
                    (setq last-button-time now)
                    (setq touch-latched true))
                nil))
        nil)
})



(defun draw-pages () {
    (if (= current-page 0) (draw-page-hypercube) nil)
    (if (= current-page 1) (draw-page-octahedron) nil)
    (if (= current-page 2) (draw-page-icosahedron) nil)
    (if (= current-page 3) (draw-page-star) nil)
})

(defun draw-touch-marker () {
    (if touch-down
        (progn
            (img-circle img touch-x touch-y 8 COLOR_TOUCH)
            (img-circle img touch-x touch-y 3 COLOR_TOUCH '(filled)))
        nil)
})

(loopwhile-thd 60 t {
    (var t-start (systime))

    (handle-buttons)

    (img-clear img COLOR_BG)

    (draw-pages)

    (draw-touch-marker)

    (draw-hud)

    (disp-render img 0 0)

    (setq fps (/ 1.0 (secs-since t-start)))
})
