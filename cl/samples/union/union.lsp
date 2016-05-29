(defparameter lstx nil)
(defparameter lsty '(a b c))
(defparameter lstz '(a c e))

(defun in-list (lst x)
  "function that check x is in list"
  (if (null lst)
      nil
      (if (eql (car lst) x)
	  t
	  (inList (cdr lst) x))))


(defun my-union(lst1 lst2 lst3)
  "function that union lst2 and list3 in list1"
  ;; copy list2 to list1
  (setf lst1 (copy-list lst2))

  ;; travel list3 and find different elements which are not in list2
  ;; then append it into list1
  (dolist (elem lst3)
    (if (eql (in-list lst2 elem) nil)
 	(setf lst1 (append lst1 (cons elem nil)))))

  (dolist (elem lst1)
    (format t "~A " elem))
  (format t "~%"))
	

(my-union lstx lsty lstz)
