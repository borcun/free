#!/usr/bin/ruby

# this is source code of pattern-sting match.

# ASCII code start of 0 digit
$DIGIT_ACODE = 48

# function that matches pattern to sting
def match(pattern, sting)
  _sting = String.new
  curr = String.new

  # process each element of pattern and convert it to new sting
  pattern.each_char do |c|
    # if character is a letter, add it _sting directly.
    # store character in current character to process next iteration
    if c =~ /[A-Za-z]/ then
      _sting += c
      curr = c
    # if character is digit, add current character times digit - 1
    # cause of -1, the character was always written above if statement one time.
    elsif c =~ /[0-9]/
      if c.ord == $DIGIT_ACODE then
        _sting.chop
        puts "Now sting #{_sting}"
      # ord function gets ASCII integer value of character
      else
        _sting += (curr * (c.ord - $DIGIT_ACODE - 1))
      end
    end
  end

  puts "_sting: #{_sting}"

  if _sting == sting then
    return true
  end

  return false
end
# end of match function

# chomp function trim new line character
puts "Enter a pattern"
pattern = gets.chomp
puts "Enter a matched sting"
sting = gets.chomp

if match(pattern, sting) then
  puts "pattern matches sting"
else
  puts "pattern doesn't match sting"
end
