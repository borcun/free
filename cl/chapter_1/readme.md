== CHAPTER 1 ==

1. Lisp is an interactive language. If you type an expression into the toplevel, Lisp will display its value. (REPL)
   
2. Lisp programs consist of expressions. An expression can be an atom, or a list of an operator followed by 
zero or more arguments. Prefix syntax means that operators can take any number of arguments.

3. The evaluation rule for Common Lisp function calls: evaluate the arguments left to right, and pass them to 
the function denoted by the operator. The quote operator has its own evaluation rule, which is to return the 
argument unchanged.

4. Along with the usual data types, Lisp has symbols and lists. Because Lisp programs are expressed as lists, 
it's easy to write programs that write programs.

5. The three basic list functions are cons, which builds a list; car, which returns the first element; and cdr, 
which returns everything after the first element.

6. In Common Lisp, t represents true and nil represents false. In a logical context, anything except nil counts 
as true. The basic conditional is if. The and and or operators resemble conditionals.

7. Lisp consists mainly of functions. You can define new ones with defun.

8. A function that calls itself is recursive. A recursive function should be considered as a process rather than 
a machine.

9. Parentheses are not an issue, because programmers read and write Lisp by indentation.

10. The basic I/O functions are read, which includes a complete Lisp parser, and format, which generates output 
based on templates.

11. You can create new local variables with l e t , and global variables with defparameter.

12. The assignment operator is setf. Its first argument can be an expression.

13. Functional programming, which means avoiding side-effects, is the dominant paradigm in Lisp.

14. The basic iteration operator is do.

15. Functions are regular Lisp objects. They can be passed as arguments, and denoted by lambda expressions.

16. In Lisp, values have types, not variables.
