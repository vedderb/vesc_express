(import "pkg::jpg_test_320_240@://vesc_packages/lib_files/files.vescpkg" 'jpg_test_320_240)

(loopwhile (not (main-init-done)) (sleep 0.1))

(import "pkg@://vesc_packages/lib_tca9535/tca9535.vescpkg" 'tca9535)
(read-eval-program tca9535)

(tca9535-init 0x20 ''rate-100k 10 9)

; Set initial values before setting directions
(tca9535-write-pins
    '(0 0) ; D_RES
    '(1 0) ; DISP_LED
    '(2 1) ; 3V3_EN
    '(3 0) ; BTN EXT
)

(tca9535-set-dir
    '(0 out) ; D_RES
    '(1 out) ; DISP_LED
    '(2 out) ; 3V3_EN
    '(3 out) ; BTN EXT

    '(4 in) ; Hazard
    '(5 in) ; Mode1
    '(6 in) ; Mode2
    '(7 in) ; Mode3
    '(10 in) ; Mode4
    '(11 in) ; Mode5
    '(12 in) ; BT1
    '(13 in) ; BT2
    '(14 in) ; BT3
    '(15 in) ; BT4
    '(16 in) ; BT5
    '(17 in) ; BT6
)

; Reset both screens and switch on backlight when done
; to avoid flicker
(defun init-displays () {
        (tca9535-write-pins '(0 0))
        (sleep 0.01)
        (tca9535-write-pins '(0 1))
        (sleep 0.2)
        (sel-disp-left)
        (disp-reset)
        (ext-disp-orientation 3)
        (sel-disp-right)
        (disp-reset)
        (ext-disp-orientation 1)
        (tca9535-write-pins '(1 1))
})

(init-displays)

(def col1 0)
(def col2 0)
(def colors (list 0xff0000 0x00ff00 0x0000ff))

(defun inc-wrap (val max) (set val (if (< (eval val) (- max 1)) (+ (eval val) 1) 0)))

(def bt1 0)
(def bt2 0)
(def bt3 0)
(def bt4 0)
(def bt5 0)
(def bt6 0)

(loopwhile-thd 100 t {
        (var buttons (tca9535-read-pins 14 13 12 15 16 17))

        (setq bt1 (if (= (ix buttons 0) 1) (+ bt1 1) 0))
        (setq bt2 (if (= (ix buttons 1) 1) (+ bt2 1) 0))
        (setq bt3 (if (= (ix buttons 2) 1) (+ bt3 1) 0))
        (setq bt4 (if (= (ix buttons 3) 1) (+ bt4 1) 0))
        (setq bt5 (if (= (ix buttons 4) 1) (+ bt5 1) 0))
        (setq bt6 (if (= (ix buttons 5) 1) (+ bt6 1) 0))
        (def light (get-adc 0))

        (if (= bt1 1) {
                (sel-disp-left)
                (disp-clear (ix colors col1))
                (inc-wrap 'col1 (length colors))
        })

        (if (= bt2 1) {
                (sel-disp-left)
                (disp-clear 0x000000)
        })

        (if (= bt3 1) {
                (sel-disp-left)
                (disp-render-jpg jpg_test_320_240 0 0)
        })

        (if (= bt4 1) {
                (sel-disp-right)
                (disp-clear (ix colors col2))
                (inc-wrap 'col2 (length colors))
        })

        (if (= bt5 1) {
                (sel-disp-right)
                (disp-clear 0x000000)
        })

        (if (= bt6 1) {
                (sel-disp-right)
                (disp-render-jpg jpg_test_320_240 0 0)
        })

        (sleep 0.1)
})
