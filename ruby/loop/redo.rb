#!/usr/bin/ruby

# redo restart loop
for i in 0..10
  if i > 5 then
    puts "#{i}"
    redo
  end
end
