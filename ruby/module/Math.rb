#!/usr/bin/ruby

# this file is mathematic module

module Math
  MY_PI = 3.14

  def Math.power(base, pow)
    res = 1
    for i in 0...pow
      res *= base
    end
    return res
  end

  def Math.perimeter(r)
    2 * MY_PI * r
  end
end
