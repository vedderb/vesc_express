(esp-now-start)

; Data contains an array with src, dst and content
(defun proc-data (data)
    (let (
            (src (map (fn (x) (bufget-u8 data x)) (range 0 6)))
            (des (map (fn (x) (bufget-u8 data x)) (range 6 12)))
            (content (map (fn (x) (bufget-u8 data x)) (range 12 (buflen data))))
        )
        (progn
            (print (list "src:" src "des:" des "content:" content))
            (free data)
)))
        
(defun event-handler ()
    (loopwhile t
        (recv
            ((event-esp-now-rx . (? data)) (proc-data data))
            (_ nil)
)))

(event-register-handler (spawn event-handler))
(event-enable 'event-esp-now-rx)
