(defun enigma (x)
  (or (not (null x))
    (or (null (car x))
        (enigma (cdr x)))))