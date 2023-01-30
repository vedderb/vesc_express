(esp-now-start)

; Broadcast address
(def other-peer '(255 255 255 255 255 255))

(esp-now-add-peer other-peer)

(loopwhile t
    (progn
        (esp-now-send other-peer "Hello")
        (sleep 1)
))
