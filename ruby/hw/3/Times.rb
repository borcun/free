#!/usr/bin/ruby

# Time Class
class Times
  #constructor
  def initialize(hour=0, minute=0, second=0)
    @hour, @minute, @second = hour, minute, second
  end

  # function that sets hour
  def setHour(hour)
    @hour = hour
  end

  # function that sets minute
  def setMinute(minute)
    @minute = minute
  end

  # function that sets second
  def setSecond(second)
    @second = second
  end

  # function that gets hour
  def getHour
    @hour
  end

  # function that gets minute 
  def getMinute
    @minute
  end

  # function that gets second
  def getSecond
    @second
  end

  def >(time)
    @hour * 3600 + @minute * 60 + @second >
      time.getHour * 3600 + time.getMinute * 60 + time.getSecond
  end

  def <(time)
    not (self > time)
  end

  def >(time)
    @hour * 3600 + @minute * 60 + @second ==
      time.getHour * 3600 + time.getMinute * 60 + time.getSecond
  end

  def to_s
    "#@hour:#@minute:#@second"
  end
end
