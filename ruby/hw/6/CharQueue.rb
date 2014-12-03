#!/usr/bin/ruby

# CharQueue Class
class CharQueue
  # constructor
  def initialize
    @str = String.new
  end

  # function that adds a character to end of string
  def add(chr)
    @str += chr
  end

  # function that adds a character to end of string
  def +(chr)
    @str += chr
  end

  def del
    @str.chop!
  end

  def -
    @str.chop!
  end

  # function that formats object
  def to_s
    @str
  end

  # function that prints object
  def <<
    puts @str
  end
end

cqueue = CharQueue.new
cqueue.add "x"
cqueue.add "x"
cqueue + "x"
puts cqueue
cqueue.del
cqueue.-
cqueue.<<
