#!/usr/bin/ruby

# Rational Number Class
class Ratio
  # default constructor
  def initialize
    @nominator = 0
    @denominator = 0
  end

  # function that sets nominator
  def setNominator(nominator)
    @nominator = nominator
  end

  # function that sets denominator
  def setDenominator(denominator)
    @denominator = denominator
  end

  # function that gets nominator
  def getNominator
    @nominator
  end

  # function that gets denominator
  def getDenominator
    @denominator
  end

  # function that prints formatted ratio
  def to_s
    "#@nominator / #@denominator"
  end

  # function that calculates floating-point result of ratio
  def to_r
    "#{@nominator.to_f / @denominator.to_f}"
  end

  # function that checks whether first rational number is bigger
  def >(ratio)
    self.to_r > ratio.to_r
  end

  # function that checks  whether first rational number is smaller
  def <(ratio)
    self.to_r < ratio.to_r
  end

  # function that checks whether two rational number equals
  def ==(ratio)
    self.to_r == ratio.to_r
  end
end
