#!/usr/bin/ruby

var = 3

# standart unless else statement
unless var < 5 then
  puts "#{var} is bigger than 5"
else
  puts "#{var} is smaller than 5"
end

# Ruby-style unless statement
puts "#{var} is smaller than 5" unless var > 5
