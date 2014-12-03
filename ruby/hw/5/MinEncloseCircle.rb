#!/usr/bin/ruby

# Point Class
class Point
  # constructor
  def initialize(x=0, y=0)
    @x, @y = x, y
  end

  # function that sets X coordinate
  def setX(x)
    @x = x
  end

  # function that sets Y coordinate
  def setY(y)
    @y = y
  end

  # function that gets X coordinate
  def getX
    @x
  end

  # function that gets Y coordinate
  def getY
    @y
  end

  # function that prints coordinates
  def to_s
    "#@x, #@y"
  end
end

# Minimum Enclosing Circle Class
class MinCircle
  # default constructor
  def initialize(points)
    @points = points
  end

  # function that calculates minimum enclosing circle center
  def center
    total_x, total_y = 0, 0

    for i in 0...@points.length
      total_x += @points[i].getX
      total_y += @points[i].getY      
    end

    @aver_x = total_x / @points.length
    @aver_y = total_y / @points.length    

    return @aver_x,  @aver_y 
  end

  # function that finds minimum enclosing circle
  def find
    # estimate center of mass of points
    self.center
    px, py = 0, 0
    diff = 0

    for i in 0...@points.length
      if (@points[i].getX - @aver_x).abs > diff
    
  end
end

# point array
points = Array.new(10)

for i in 0...10
  points[i] = Point.new(i*3, i*2)
end

circle = MinCircle.new(points)

x, y = circle.center
puts "#{x}, #{y}"
