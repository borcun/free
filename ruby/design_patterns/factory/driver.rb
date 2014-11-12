#! /bin/ruby -w

$LOAD_PATH << "."

require "CoffeeMaker.rb"

coffee_maker = CoffeeMaker.instance
coffee_maker.create("turkish")
coffee_maker.create("nescafe", 2, "yes")
coffee_maker.print
