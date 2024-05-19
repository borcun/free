(define make-stack
  (lambda ()
    (let ((ls '()))
      (lambda (msg . args)
	(cond
	 ((eqv? msg 'empty?) (null? ls))
	 ((eqv? msg 'push!) (set! ls (cons (car args) ls)))
	 ((eqv? msg 'top) (car ls))
	 ((eqv? msg 'pop!) (set! ls (cdr ls)))
	 (else "invalid operation"))))))
