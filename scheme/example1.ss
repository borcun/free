(list (cons 'a 'b) (list (list 'c 'd)) ())
(let ((x 2) (y 5)) (+ x y))

(cons (car (list 'a 'b 'c)) (cdr (cdr (list 'a 'b 'c))))

((lambda (x) (if (> 10 x)
		 (* x x)
		 (+ x x))) 12)

(let ([f (lambda (x) x)]) (f 'a))

(pair? '(a . b))
