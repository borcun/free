#!/usr/bin/ruby

# Vehicle class
class Vehicle
  @@num_of_vehicle = 4;

  def increase
    @@num_of_vehicle += 1;
  end

  def print_var
    puts "Vehicle count: #@@num_of_vehicle"
  end
end

vec1 = Vehicle.new
vec2 = Vehicle.new

vec1.increase
vec1.print_var
vec2.increase
vec2.print_var
