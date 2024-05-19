(define make-queue
  (lambda ()
    (let ((end (cons 'ignored '())))
      (cons end end))))
