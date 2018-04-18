#!/bin/env lua

function rev( str )
   if 1 == str:len() then
      print( str:sub( str:len() ) )
      return
   end
   
   io.write( str:sub( str:len() ) )
   rev( str:sub( 1, str:len()-1 ) )
end

str = "hello world"
rev( "hello world" )
