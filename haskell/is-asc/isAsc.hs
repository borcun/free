isAsc :: [Int] -> Bool
isAsc [] = True
isAsc (x:xs) = if x > isAsc xs then False else isAsc xs
