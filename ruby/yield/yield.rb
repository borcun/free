#!/usr/bin/ruby

# yield usage

def test
  yield "Burak"
  puts "yield burak"
  yield "Orcun"
  puts "yield Orcun"
  yield "Ozkablan"
end

test {|name| puts "Name is #{name}"}
