(rgbled-init 43 1)
(def led (rgbled-buffer 1 0))
(rgbled-color led 0 0x080800)
(rgbled-update led)

(loopwhile-thd 100 t {
    (gpio-write 60 1)
    (gpio-write 61 0)
    (sleep 0.25)
    (gpio-write 60 0)
    (gpio-write 61 1)
    (sleep 0.25)
})
