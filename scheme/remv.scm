(define remv
  (lambda (elem ls)
    (if (null? ls)
	()
	(if (eqv? elem (car ls))
	    (remv elem (cdr ls))
	    (cons (car ls) (remv elem (cdr ls)))))))
