(define memv
  (lambda (ls elem)
    (if (null? ls)
	#f
	(if (eqv? (car ls) elem)
	    ls
	    (memv (cdr ls) elem)))))
