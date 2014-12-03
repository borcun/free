#!/usr/bin/ruby

$LOAD_PATH << "."

require "Ratio.rb"
require "Times.rb"

ratio1 = Ratio.new
ratio2 = Ratio.new

ratio1.setNominator 5
ratio1.setDenominator 19

ratio2.setNominator 5
ratio2.setDenominator 10

if ratio1 > ratio2 then
  puts "#{ratio1.to_r} is bigger than #{ratio2.to_r}"
else
  puts "#{ratio2.to_r} is bigger than #{ratio1.to_r}"
end

time1 = Times.new(11, 55, 10)
time2 = Times.new(12, 33, 42)

if time1 < time2
  puts "#{time2.to_s} is bigger than #{time1.to_s}"
end

