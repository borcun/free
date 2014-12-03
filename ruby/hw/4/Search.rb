#!/usr/bin/ruby

$LOAD_PATH << "."

require "Ratio.rb"
require "Times.rb"

# Search module
module Search
  # function that performs linear search algorithm
  def Search.linear(arr, key)
    for i in 0...arr.length
      if arr[i] == key
        # if key found, return found index
        return i
      end
    end
    # if key not found, return -1
    return -1
  end

  # function that performs binary search algorithm
  def Search.binary(arr, key)
    middle = arr.length / 2
    
    if middle == 0 then
      if arr[middle] == key then
        return true
      end
      return false
    elsif arr[middle] > key
      return Search.binary(arr[0...middle], key)
    elsif arr[middle] < key      
      return Search.binary(arr[middle...arr.length - 1], key)      
    else
      return true
    end
  end
end
