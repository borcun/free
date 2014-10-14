#!/usr/bin/ruby

$i = 0

# this loop is a bit different to while
# this different is that loop runs in limit value too, so for 10.
# the second one is that condition reverses, so '<' in while to '>'.
until $i > 10 do
  puts "number: #$i"
  $i += 1
end

# ! by the way, until can be used with begin and end block as while.
