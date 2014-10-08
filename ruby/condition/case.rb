#!/usr/bin/ruby

var = 3

case var
  when 1
  puts "var is #{var}"
  when 2
  puts "var is #{var}"
  else
  puts "var is #{var}"
end

case var
  when 0...3
  puts "var is between 0 and 3"
  when 3...6
  puts "var is between 3 and 6"
  when 6...9
  puts "var is between 6 and 9"
  else 
  puts "var is bigger than 10"
end
