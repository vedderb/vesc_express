(esp-now-start)

; Data contains an array with src, dst and content
(defun proc-data (data)
    (progn
        (def src (map (fn (x) (bufget-u8 data x)) (range 0 6)))
        (def des (map (fn (x) (bufget-u8 data x)) (range 6 12)))
        
        ; Content as list
        (def content (map (fn (x) (bufget-u8 data x)) (range 12 (buflen data))))
        
        ; Content as array
;        (def content (array-create (- (buflen data) 12)))
;        (bufcpy content 0 data 12 (buflen content))

        (print (list "src:" src "des:" des "content:" content))
        (free data)
;        (free content)
))
        
(defun event-handler ()
    (loopwhile t
        (recv
            ((event-esp-now-rx . (? data)) (proc-data data))
            (_ nil)
)))

(event-register-handler (spawn event-handler))
(event-enable 'event-esp-now-rx)
