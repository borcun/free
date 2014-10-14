#!/usr/bin/ruby

# =====================================================
# method without parameter
def hello_world
  puts "Hello World"
end

hello_world
puts

# =====================================================
# method with parameter
def say_hello(name)
  puts "Hello #{name}"
end

say_hello "burak"
puts

# =====================================================
# method with default parameter
def message(name="burak")
  puts "You have a message, #{name}"
end

message
message "orcun"
puts

# =====================================================
# method with more parameters than one
def dialog(name1, name2)
  puts "#{name1} and #{name2} are talking"
end

dialog "burak", "orcun"
dialog "burak", "orcun"
puts

# =====================================================
# return value of a method is its last statement
def retLast
  i = 1
  j = 5
end

$ret = retLast
puts "return value: #$ret"; 

# =====================================================
# multiple return values
def multRets
  i = 1
  j = 5
  k = 8
  return i,j,k
end

$ret = multRets
puts "Multiple Return Values"
puts "#$ret"
puts

# =====================================================
# elipsis operator limitless parameter (actually, not limitless)
def unlimit(*param)
  puts "parameters count: #{param.length}"
  for i in 0...param.length
    puts "#{i}. parameter: #{param[i]}"
  end
end

unlimit "burak", "orcun", 1, 4.2
