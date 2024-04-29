(define shorter?
  (lambda (ls1 ls2)
    (cond
     ((and (null? ls1) (null? ls2)) 0)
     ((and (not (null? ls1)) (null? ls2)) -1)
     ((and (null? ls1) (not (null? ls2))) 1)
     (else (shorter? (cdr ls1) (cdr ls2))))))

(define short
  (lambda (ls1 ls2)
    (if (eqv? (shorter? ls1 ls2) -1)
	ls2
	ls1)))
  
