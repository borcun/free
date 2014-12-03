#!/usr/bin/ruby

arr = Array.new(5) { |i| i = 0}
array = ["burak", 28, 3.14, "orcun"]
digits = Array(0...10)
x = "-----------------"

array.each do |elem|
  puts elem
end

puts x

for i in 0...arr.length
  puts "#{arr[i]}"
  arr[i] = i
end

puts

for i in 0...arr.length
  puts "#{arr.at(i)}"
end
