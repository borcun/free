sumtorial :: Integer -> Integer
sumtorial 0 = 0
sumtorial n = n + sumtorial (n - 1)

isEven :: Integer -> Bool
isEven n
  | n `mod` 2 == 0 = True
  | otherwise = False

f :: Int -> Int -> Int -> Int
f x y z = x + y + z

main = do
  print (sumtorial 10)
  print (isEven 1)
  print (isEven 2)
  print (f 3 4 5)
