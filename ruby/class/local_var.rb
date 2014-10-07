#!/usr/bin/ruby

class Vehicle
  CONST_VAR = 8;

  def method1
    local_var = 4
    puts "Local Variable: #{local_var}"
    puts "Constant Data Member: #{CONST_VAR}"
  end
end

vec1 = Vehicle.new
vec2 = Vehicle.new

vec1.method1
vec2.method1
