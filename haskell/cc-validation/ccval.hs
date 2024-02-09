-- function that converts an integer to a list including digits of the integer
toDigits :: Int -> [Int]
toDigits num
  | num < 10 = [num]
  | otherwise = (toDigits (num `div` 10)) ++ [num `mod` 10]

-- function that doubles numbers of the list, whose indices are odd
toDoubleEveryOther :: [Int] -> [Int]
toDoubleEveryOther nums
  | 1 == length(nums) = [2 * (head nums)]
  | 0 == (length nums) `mod` 2 = (toDoubleEveryOther (init nums)) ++ [last nums]
  | otherwise = (toDoubleEveryOther (init nums)) ++ [2 * last nums]

-- a utility function that gets sum of digits of an integer
sumDigitsUtil :: Int -> Int
sumDigitsUtil num
  | num < 10 = num
  | otherwise = (num `mod` 10) + sumDigitsUtil (num `div` 10)

-- function that gets sum of digits processed of the credit card
sumDigits :: [Int] -> Int
sumDigits nums
  | 1 == (length nums) = sumDigitsUtil (head nums)
  | otherwise = sumDigitsUtil (head nums) + sumDigits (tail nums)

-- validation function for the credit card
validate :: Int -> Bool
validate num = 0 == (sumDigits (toDoubleEveryOther (toDigits num))) `mod` 10

main = do
  putStr ("4012888888881881 => ")
  print (validate 4012888888881881)
  putStr ("4012888888881882 => ")
  print (validate 4012888888881882)
