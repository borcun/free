#!/bin/env lua

function ispali( str )
   return str:sub( 1, str:len() // 2 + ( str:len() % 2 ) ):reverse() == str:sub( str:len() // 2 + 1 )
end

edip1 = "ey edi adanada pide ye"
edip2 = "ey edip adanada pide ye"
helleh1 = "helleh"
helleh2 = "helleha"

print( ispali( edip1 ) )
print( ispali( edip2 ) )
print( ispali( helleh1 ) )
print( ispali( helleh2 ) )
