
(def arr (mkarray 4))

(setix arr 0 (list 1 2 3))
(setix arr 1 (list 4 5 6))
(setix arr 2 (list 7 8 9))
(setix arr 3 12b)

(gc)

(def flat (flatten arr))

(gc)

(def unflat (unflatten flat))

(gc)

(check (and (eq (ix unflat 0) (list 1 2 3))
            (eq (ix unflat 1) (list 4 5 6))
            (eq (ix unflat 2) (list 7 8 9))
	    (eq (ix unflat 3) 12b)))
