myElem [] val = False
myElem (x:xs) val = if x == val then True else myElem xs val
