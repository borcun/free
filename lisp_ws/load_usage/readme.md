load function loads script files in interpreter, then all functions
which are in script file can be used directly.

command
-------

;; run own interpreter
sbcl

;; load script file. it hasn't to be relative path.
(load "power.lisp")


;; now, functions are available for usage.
(second_power 4)
