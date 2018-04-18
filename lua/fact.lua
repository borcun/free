#!/usr/bin/env lua5.3

function factorial( n )
   if 2 > n then
      return 1
   end

   return n * factorial( n-1 )
end

n = 5
io.write( "fact(5): ", factorial(n), "\n" )
