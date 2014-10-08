What did I see in Ruby?
======================

 When i am learning Ruby, I notice some details of Ruby which are inherited from
other programming languages such as C, C++, JAVA, LISP, AWK, Python. Because I am
familiar these languages as eihter professional or beginner, Ruby isn't hard to
learn for me. In addition, I like it so much compared to Python, LISP or Rust.

 I wanna mention about its some features which are inspired from other languages.

 ==1. Hello World==
  puts and print functions are like Python's.
  
  puts "Hello World"
  print "Hello World"

  By the way, the language doesn't need semicolon if the line has one statement.
  This feature is from Python, too.

 2. BEGIN and END Statements
 ---------------------------
  This features remind me AWK.

  BEGIN {
    statements 1
    statements 2
		...
    statements n
  }

  END {
    statements 1
    statements 2
		...
    statements n
  }

  3. Comments
  -----------
	 The one line comment style is like Python, # operator comments the line.
 
   # comment

  4. Variables
	------------
	 The variables in Ruby are special to self, I haven't seen familiar variable
	 creation in any language. global, local, class or constant variables are for
	 special to Ruby wholely.

  5. Function Declaration
	-----------------------
	 The def keyword declares a function. It is from Python and Common LISP. In some
	 LISP derivations, defun, defn keywords are used but Common LISP uses def keyword
	 as Python uses.

	 def foo
	 		 puts "foo function"
	 end

	 In addition, there isn't parenthesises, no indentation rules as Python but the
	 function ends with end keyword Python uses.

	6. Constructor
	--------------
	 Because Ruby has garbage collector, it doesn't need destructor. In C++ and JAVA,
	 constructor name must be same with class name, In Python __init__ name is reserved
	 for constructor. In Ruby, initialized name is for constructor. I think, C++ and 
	 JAVA are the best in this manner and Ruby is better than Python. initialize word
	 is more suitable for a constructor naming instead of __init__.
	  
  7. Pseudo-Variables
	-------------------
	 Ruby takes the easy way out in these variables. self is from Python, true and 
	 false are universal, nil is from LISP, __FILE__ and __LINE__ are from C.

  8. Arrays and Hashes
	--------------------
	 The Ruby arrays and hashes data structures are so easy to many languages. The
	 extra feature of these data structures which they can contains different data
	 types as element. There are parts which resemble to Python or PHP such as index
	 operator when creating array and => operator when creating hash. Though all, the
	 Ruby has new features about its array and hash.

	 each do statement is awesome, you don't need any loop to travel data structures.

  9. Operators
	   ----------
	 In many languages, the arithmetic operators are same except exponent operator.
	 Python use ** operator to calculate power of a number, Ruby too.

	 The some comparison operators of Ruby are designed so cleverly. Design logics may
	 be taken from LISP and C, even so they make so easy some situations.

	 <=> operator doesn't only performs like strcmp in C but also it can compares all
	 data types except string. Two operands are same, it returns 0, the first one is
	 bigger than second, its returns 1, otherwise -1.

	 === operator is used to check range. For instance, (1..10) === 7 statement controls
	 that 7 is in between 1 and 10. I admire operator logic. It is so necessary, why
	 didn't it be though before?

	 .eql? and equal? operators are be familir Common LISP eql and equal operators. 
	 The .eql? compare both data contents and types. For example, 1.eql?1.0 statement
	 returns false, because data types integer and float, they are different. The equal?
	 performs like Common LISP, it returns true only if the operands are same objects.

	 The compound assignment and bitwise operators are same with C.

	 The logical operators are same Python, they can be used both sign (||) and word (or).

	 In addition, here is parallel assignment of Python.
	 a, b = 1, 2  or  a, b = b, a

	 The ternary condition operator is from C.

	 The range operators .. and ... make easy some operations. .. operator is in Python too.