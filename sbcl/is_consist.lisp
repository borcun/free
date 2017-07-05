(defun is-consist (obj lst)
  (if (null lst)
      0
      (if (eql obj (car lst))
	  lst
	  (is-consist obj (cdr lst)))))
