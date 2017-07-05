`(defun print-ast-rec (x)
   (if (not (eq 0 x))
       (progn (format t "*")
	      (print-ast-rec(- x 1)))
       (format t "~%")))


(defun print-ast-it (x)
  (do ((i 1 (+ i 1)))
      ((> i x) (format t "~%"))
    (format t "*")))


(defun count-a-rec (lst)
  (if (null lst)
      0
      (if (eq (car lst) 'a)
	  (+ 1 (count-a (cdr lst)))
	  (count-a (cdr lst)))))


(defun count-a-it (lst)
  (let ((x 0))
    (dolist (obj lst)
      (if (eq obj 'a)
	  (setf x (+ 1 x))))x))
