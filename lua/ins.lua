#!/bin/env lua

function insert( str1, pos, str2 )
   if pos < 1 or pos > str1:len() then
      print( "invalid position:", pos )
      return str1
   end

   return str1:sub(1, pos-1) .. str2 .. str1:sub(pos)
   
end

print( insert( "hello world", 1, "start: " ) )
print( insert( "hello world", 7, "sweet " ) )
