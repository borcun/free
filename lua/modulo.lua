#!/usr/bin/env lua

function modulo( num, mod )
   if num < mod then
      return num
   else
      return modulo( num-mod, mod )
   end
end

io.write( "0 % 3: ", modulo( 0, 3 ), "\n" )
io.write( "1 % 3: ", modulo( 1, 3 ), "\n" )
io.write( "3 % 3: ", modulo( 3, 3 ), "\n" )
io.write( "10 % 3: ", modulo( 10, 3 ), "\n" )
io.write( "11 % 3: ", modulo( 11, 3 ), "\n" )
io.write( "12 % 3: ", modulo( 12, 3 ), "\n" )
