#!/usr/bin/ruby

# if method doesn't denote with any accessor keyword, it is public by default.

# if a function is out of class, it is private for class attributes and methods.
def hello_world
  puts "Hello World"
end

# alias keyword resembles typedef of C++ or some like define macro of C
alias hello hello_world
hello

# undef undefines defined method
undef hello
hello

class ATM
  # this method can be reached by an instance of class
  def account
  end

  # this method is global method between instances of class
  # it can be reached with class name and dot operator
  def ATM.credit
  end
end
