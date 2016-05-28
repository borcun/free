(defun list-in-list (alist)
  (if (null alist)
    nil
    (if (listp (car alist))
          t
          (list-in-list (cdr alist)))))