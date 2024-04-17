(rgbled-init 20)

(def strip1 (rgbled-buffer 10 2))
(def strip2 (rgbled-buffer 10 0))

(rgbled-color strip1 0 0x330000)
(rgbled-color strip1 1 0x22000000u32)
(rgbled-color strip1 9 0x003311)

(rgbled-color strip2 0 0x330000)
(rgbled-color strip2 1 0x22000000u32)
(rgbled-color strip2 6 0x330011)

(rgbled-update strip1)
(rgbled-init 21)
(rgbled-update strip2)
