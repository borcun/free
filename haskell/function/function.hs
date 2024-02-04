sumtorial :: Integer -> Integer
sumtorial 0 = 0
sumtorial n = n + sumtorial (n - 1)

isEven :: Integer -> Bool
isEven n
  | n `mod` 2 == 0 = True
  | otherwise = False

main = do
  print (sumtorial 10)
  print (isEven 1)
  print (isEven 2)
