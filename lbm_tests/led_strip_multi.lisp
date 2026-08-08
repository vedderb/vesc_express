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
; Bench script: wire strips to pin1 and pin2 below, run it, and follow the
; console - each step prints what you should be seeing. Retarget by changing
; the pin and length defs; avoid pins the hw config already uses (on the C3
; devkit the UART is 20/21, and 12-17 are flash).

(def pin1 2)
(def pin2 4)

(def len1 10)
(def len2 10)

(def strip1 (rgbled-buffer len1 0)) ; GRB
(def strip2 (rgbled-buffer len2 0)) ; GRB

; Light every pixel, so a wrong length shows up as a dark tail rather than
; looking correct.
(defun fill-strip (strip len color)
        (looprange i 0 len (rgbled-color strip i color)))

; Zero every pixel and push it, so the strip actually goes dark. Writing the
; colours (rather than clearing the whole array) preserves byte 0, which holds
; the LED type and gamma flag that rgbled-buffer put there.
(defun clear-strip (strip pin len) {
        (fill-strip strip len 0)
        (rgbled-update strip pin)
})

; Distinct colours so the two strips are told apart at a glance.
(defun paint () {
        (fill-strip strip1 len1 0x330000) ; dim red
        (fill-strip strip2 len2 0x003311) ; dim teal
})

(paint)

; --- Two strips lit at the same time ---------------------------------------
; Both stay registered; the second init does not tear the first one down.
(rgbled-init pin1)
(rgbled-init pin2)

(rgbled-update strip1 pin1) ; second arg selects the strip
(rgbled-update strip2 pin2)
(print (str-merge "pin " (to-str pin1) " fully red, pin " (to-str pin2) " fully teal"))
(sleep 2)

; --- Release one, keep the other -------------------------------------------
; Mode 1 holds the released pin LOW, mode 0 resets it. The second argument
; picks a single pin; without it every registered strip is released.
; Clear first, otherwise pin1 would keep displaying its last frame.
(clear-strip strip1 pin1 len1)
(rgbled-deinit 1 pin1)
(print (str-merge "pin " (to-str pin1) " dark and released, pin " (to-str pin2) " still lit"))
(sleep 2)

; --- Re-init the released pin ----------------------------------------------
(paint)
(rgbled-init pin1)
(rgbled-update strip1 pin1)
(print (str-merge "pin " (to-str pin1) " lit again, pin " (to-str pin2) " unaffected"))
(sleep 2)

; --- Arg-less update targets the most recent rgbled-init -------------------
; This is what keeps single-strip scripts working unchanged.
(rgbled-init pin2)
(rgbled-update strip2)
(print (str-merge "pin " (to-str pin2) " refreshed through the arg-less update"))
(sleep 2)

; --- Release everything ----------------------------------------------------
(clear-strip strip1 pin1 len1)
(clear-strip strip2 pin2 len2)
(rgbled-deinit 1)
(print "both strips dark and released")
