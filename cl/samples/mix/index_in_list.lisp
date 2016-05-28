; function that checks whether the first parameter is in the second parameter list
; @x - symbol
; @y - list
(defun index-in-list (x y)
  ; if y list is null,
  (if (null y)
    ; return nil
    nil
    ; else, compare x and first element of y list
    ; if they are equal,
    (if (eql x (car y))
      ; return 0
      0
      ; else, assign result of function to z
      ; and z and z+1
      ; and operator evaluate its each parameter from left to right
      ; if one of them is nil, return nil without evaluate next
      ; if all of them aren't nil, return its last parameter
      ; so, result of (and 0 any-number) is any-number
      (let ((z (in_list x (cdr y))))
        (and z (+ z 1))))))