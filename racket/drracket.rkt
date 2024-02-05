;; The first three lines of this file were inserted by DrRacket. They record metadata
;; about the language level of this file in a form that our tools can easily process.
#reader(lib "htdp-beginner-reader.ss" "lang")((modname drracket) (read-case-sensitive #t) (teachpacks ()) (htdp-settings #(#t constructor repeating-decimal #f #t none #f () #f)))
(check-expect (add 2 3) 5)
(define (add x y) (+ x y))

(check-expect (square 4) 16)
(define (square x) (* x x))

(define (fact x)
  (if (< x 2) 1 (* x (fact (- x 1)))))

(add 5 10) ; add function
(square 5)
(fact 5)