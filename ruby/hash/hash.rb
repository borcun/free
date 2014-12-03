#!/usr/bin/ruby

k = v = { "One" => 1, "Two" => 2, "Three" => 3 }
day = Hash.new("day")

k.each do |key, value|
  print key, " is ", value, "\n"
end

puts "#{day[1]}"
