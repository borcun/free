#!/usr/bin/ruby

$LOAD_PATH << "."

require "CharQueue.rb"

cqueue = CharQueue.new

cqueue = cqueue + "x"
cqueue = cqueue + "x"
cqueue = cqueue + "x"

puts cqueue

cqueue.del
puts cqueue
