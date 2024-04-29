(define make-list
  (lambda (count obj)
    (if (eqv? 0 count)
	()
	(cons obj (make-list (- count 1) obj)))))
