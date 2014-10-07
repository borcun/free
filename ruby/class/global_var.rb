#!/usr/bin/ruby

$global_var = 10

class Class1
  def print_global
    puts "global var: #$global_var"
  end
end

class Class2
  def print_global
    puts "global var: #$global_var"
  end
end

obj1 = Class1.new
obj2 = Class2.new

obj1.print_global
obj2.print_global
