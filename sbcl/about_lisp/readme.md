# LISP programs run either on an interpreter or compiled code.
# LISP expressions are called symbolic expression or s-expression.
# LISP uses prefix notation.
# LISP programs are made up of three building blocks:
  atom, list and string
  - atom is a number or string of contiguos characters.
  - list is sequence of atoms and/or other lists.
  - string is character group.
# ; is comment syntax of LISP.
# LISP evaluates everything, except numbers, t (true/false) and nil.
# LISP variables consist of any number and character, except whitespace,
  open/close parentheses, double and single quotes, backslash, comma,
  semicolon and vertical bar.
# Single quotation mark is used to not evaluate form.
# defvar function defines a variable. 
  ; (defvar x 5)
# Technically, a macro is a function that takes an s-expression as arguments
  and returns a LISP form, which is then evaluated. A macro is defined with
  defmacro keyword.
  ; (defmacro my-macro (x))
  ; the macro whose name is my-macro, parameter is x
# In LISP, each variable is represented by a symbol.
# In LISP, constant variable is defined with defconstant keyword.
  ; (defconstant PI 3.14)
#