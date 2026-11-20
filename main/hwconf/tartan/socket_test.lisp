(def pins '(0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19
            44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59))

(loopforeach pin pins {
    (gpio-configure pin 'pin-mode-out)
    (gpio-write pin 0)
})

(loopwhile-thd 100 t {
    (loopforeach pin pins (gpio-write pin 0))
    (sleep 0.5)
    (loopforeach pin pins (gpio-write pin 1))
    (sleep 1)
})
