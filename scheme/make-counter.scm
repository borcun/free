(define make-counter
  (let ((counter 0))
    (lambda ()
      (set! counter (+ counter 1)))))
