module Concat where

greeting :: String
greeting = "Hello" ++ " World"

name :: String
surname :: String

name = "Burak Orcun"
surname = "OZKABLAN"

main :: IO()
main = do
  putStrLn greeting
  putStrLn fullName
  where fullName = concat[name, " ", surname]
