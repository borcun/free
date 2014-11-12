#! /bin/ruby -w

$LOAD_PATH << "."

require "Coffee.rb"

# CoffeeMaker class
class CoffeeMaker
  @@instance = CoffeeMaker.new
  @@coffee_table = Hash.new

  # function that gets class instance
  def self.instance
    @@instance
  end

  # function that creates new coffee
  # name must be unique !
  def create(name, sugar=0, milk_powder="no")
    @@coffee_table[name] = Coffee.new(name, sugar, milk_powder)
  end

  # function that finds coffee in coffee table
  def find(name)
    @@coffee_table[name]
  end

  # function that prints all coffees
  def print
    @@coffee_table.each do |key, value|
      value.print
    end
  end
end
