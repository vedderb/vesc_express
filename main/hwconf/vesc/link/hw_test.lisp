(print (f-fatinfo)) ; Should show the size and free space

; Enable LED and GNSS power
(gpio-configure 7 'pin-mode-out)
(gpio-write 7 1)

; Make first led red and second led green
(rgbled-init 8)
(def led-buf (rgbled-buffer 2 0 0 1))
(rgbled-color led-buf 0 (color-make 1.0 0.0 0.0) 0.2)
(rgbled-color led-buf 1 (color-make 0.0 0.0 1.0) 0.2)
(rgbled-update led-buf)
