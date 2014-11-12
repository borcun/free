#! /bin/ruby -w

# Coffee Class
class Coffee
  # constructor
  def initialize(name, sugar=0, milk_powder="no")
    @name = name
    @sugar = sugar
    @milk_powder = milk_powder
  end

  # function that sets coffee name
  def setName=(name)
    @name = name
  end

  # function that sets sugar quantity
  def setSugar=(sugar)
    @sugar = sugar
  end

  # function that sets milk powder quantity
  def setMilkPowder=(milk_powder)
    @milk_powder = milk_powder
  end

  # function that gets coffee name
  def getName
    @name
  end

  # function that gets sugar quantity
  def getSugar
    @sugar
  end

  # function that gets milk powder quantity
  def getMilkPowder
    @milk_powder
  end
  
  # function that print coffee
  def print
    puts "The #@name coffee with #@sugar sugar and #@milk_powder milk powder"
  end
end
