#!/usr/bin/ruby

$LOAD_PATH << "."

require "pattern_sting.rb"

pm1 = PatternMatcher.new "a2b3c"
pm2 = PatternMatcher.new "a2b3c"

if pm1.match("aabbbc") then
  puts "TRUE"
else
  puts "FALSE"
end

if pm2.match("aabbc") then
  puts "TRUE"
else
  puts "FALSE"
end
