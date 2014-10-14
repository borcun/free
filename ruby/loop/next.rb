#!/usr/bin/ruby

# next keyword treats like continue in C
for i in (0..10)
  if i == 5 then
    next
  end
  puts "#{i}"
end
