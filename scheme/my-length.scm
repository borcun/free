(define my-length
  (lambda (ls)
    (if (null? ls)
	0
	(+ 1 (my-length (cdr ls))))))
