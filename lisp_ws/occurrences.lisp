(defun occurrences (lst res)
	   (if (null lst)
	       res
	       ((if (equal (assoc (car lst) res) nil)
		    ((cons (cons (car lst) 1) res)
		      (occurrences (cdr lst) res))
		    ((setf (1+ (assoc (car lst) res))))
		      (occurrences (cdr lst) res)))))