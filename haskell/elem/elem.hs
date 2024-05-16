myElem :: (Eq a) => a -> [a] -> Bool
myElem val [] = False
myElem val (x:xs) = if x == val then True else myElem val xs
