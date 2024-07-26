;(define transpose1
;  (lambda (ls1 ls2)
;    (map cons ls1 ls2)))

(define transpose
  (lambda (ls)
    (map cons (car ls))))
