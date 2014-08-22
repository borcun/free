(defun new-union (lst1 lst2)
  ; function that unions lst1 with lst2 and prevents duplication atom
  ; if lst1 list is empty, return lst2
  (if (null lst1)
    lst2
    ; if lst1 list is not empty, get first atom from lst1 and 
    ; push it to lst2 with preventing duplication and call new-union
    ; with cdr of lst1 and pushed lst2
    (new-union (cdr lst1) (pushnew (first lst1) lst2))))