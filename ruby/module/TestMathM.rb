#!/usr/bin/ruby
# this file drives mathematic module

# LOAD_PATH is a global variable to tell workspace for module
# if it isn't set, Ruby interpreter looks for default directory for Math module
$LOAD_PATH << '.'

require "Math.rb"

=begin
if Math.rb file contains more module, you can use include keyword 
for any speficic module. In this sample, include is not necessary.
So, in fact require invokes file, include invokes module
=end

include Math

$r = 5
$b = 2
$per = Math.perimeter($r)
$res = Math.power($b, $r)

puts "if r is #$r, the perimeter of circle is #$per"
puts "#$b^#$r is #$res"
puts Math::MY_PI
