(defun consist-list (lst)
  (if (null lst)
      (format t "No List")
      (if (listp (car lst))
	  (format t "OK")
	  (consist-list (cdr lst)))))