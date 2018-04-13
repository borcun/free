#!/bin/env lua

function poly( coef, x )
   local res = 0
   
   for i=1, #coef do
      res = res + coef[i] * ( x ^ ( i - 1 ) )
   end

   return res
end

coef = {4, 1, 2, 2}
print( poly( coef, 2 ) )
