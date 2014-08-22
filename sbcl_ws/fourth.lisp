(defun my-fourth (lst)
	   (if (null lst) 
	       nil
	       (car (cdr (cdr (cdr lst))))
	       ))