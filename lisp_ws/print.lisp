(defun histogram (x)
  (do ((i 0 (+ i 1))) 
    ((eql i x) nil)
    (format t "~A " '*)))