#!/usr/bin/ruby

=begin
mixin is like composition in some OO language. The class doesn't inherit a class 
exactly, use some its methods and properties instead as they belong to self.
=end

# module X
module ModuleX
  def print_x
    puts "I am print_x function of ModuleX"
  end
end

# module Y
module ModuleY
  def print_y
    puts "I am print_y function of ModuleY"
  end
end

# class Z
class Z
  include ModuleX
  include ModuleY

  def print_z
    puts "I am z function of class Z"
  end
end

z = Z.new
# print_x method belongs to ModuleX
z.print_x
# print_y method belongs to ModuleY
z.print_y
z.print_z
