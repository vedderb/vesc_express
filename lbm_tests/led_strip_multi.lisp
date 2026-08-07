; Multi-strip behaviour of the VESC Express LED driver.
;
; The single-strip driver released the previous strip whenever rgbled-init was
; called on a new pin. This driver registers an additional strip instead, so
; several pins can be driven at once and a strip is released explicitly with
; rgbled-deinit.
;
; Uses the two-argument forms of rgbled-update and rgbled-deinit, so it does
; NOT run on the single-strip driver - see led_strip.lisp for the script that
; is portable to both.
;
; Note on turning strips off: releasing a pin only stops driving data, and
; addressable LEDs latch their last frame, so a released strip keeps showing
; whatever was sent to it. Blanking requires sending a zeroed frame first,
; which is what clear-strip below does.
;
; Bench script: connect strips to pins 20 and 21, run it, and follow the
; console - each step prints what you should be seeing.

(def led-count 10)

(def strip1 (rgbled-buffer led-count 2)) ; GRBW
(def strip2 (rgbled-buffer led-count 0)) ; GRB

; Zero every pixel and push it, so the strip actually goes dark. Writing the
; colours (rather than clearing the whole array) preserves byte 0, which holds
; the LED type and gamma flag that rgbled-buffer put there.
(defun clear-strip (strip pin) {
        (looprange i 0 led-count (rgbled-color strip i 0))
        (rgbled-update strip pin)
})

(defun paint () {
        (rgbled-color strip1 0 0x330000)
        (rgbled-color strip1 9 0x003311)
        (rgbled-color strip2 0 0x330000)
        (rgbled-color strip2 6 0x330011)
})

(paint)

; --- Two strips lit at the same time ---------------------------------------
; Both stay registered; the second init does not tear the first one down.
(rgbled-init 20)
(rgbled-init 21)

(rgbled-update strip1 20) ; second arg selects the strip
(rgbled-update strip2 21)
(print "pin 20 and pin 21 both lit")
(sleep 2)

; --- Release one, keep the other -------------------------------------------
; Mode 1 holds the released pin LOW, mode 0 resets it. The second argument
; picks a single pin; without it every registered strip is released.
; Clear first, otherwise pin 20 would keep displaying its last frame.
(clear-strip strip1 20)
(rgbled-deinit 1 20)
(print "pin 20 dark and released, pin 21 still lit")
(sleep 2)

; --- Re-init the released pin ----------------------------------------------
(paint)
(rgbled-init 20)
(rgbled-update strip1 20)
(print "pin 20 lit again, pin 21 unaffected")
(sleep 2)

; --- Arg-less update targets the most recent rgbled-init -------------------
; This is what keeps single-strip scripts working unchanged.
(rgbled-init 21)
(rgbled-update strip2)
(print "pin 21 refreshed through the arg-less update")
(sleep 2)

; --- Release everything ----------------------------------------------------
(clear-strip strip1 20)
(clear-strip strip2 21)
(rgbled-deinit 1)
(print "both strips dark and released")
