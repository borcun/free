(defun duplication (x aList)
  (if (null aList)
    0
    (if (eql x (car aList))
      (+ 1 (duplication x (cdr aList)))
      (duplication x (cdr aList)))))