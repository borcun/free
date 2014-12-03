#!/usr/bin/ruby

$LOAD_PATH << "."

require "Search.rb"

arr = Array.new(10)

for i in 0..10
  arr[i] = i
end

puts "result of linear search is #{Search.linear(arr, 6)}"
puts "result of binary search is #{Search.binary(arr, 6)}"

puts "result of linear search is #{Search.linear(arr, -4)}"
puts "result of binary search is #{Search.binary(arr, 12)}"
