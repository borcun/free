#!/usr/bin/ruby

# create class
class Vehicle
  @@no_of_vehicle = 2;
  
  # this is constructor may be meant initialize function
  def initialize(id, name)
    @m_id = id;
    @m_name = name;
  end

  def hello
    puts "Hello #@m_name, your id #@m_id";
  end
end

# creating object
vec1 = Vehicle.new(1, "Burak");
vec2 = Vehicle.new(2, "Orcun");

vec1.hello();
vec2.hello();
