data BookInfo = Book Int String [String]
  deriving Show

data Person = Person String Int
  deriving Show

getAge :: Person -> Int
getAge (Person _ age) = age

main = do
  let book = Book 1 "How to program in Haskell" ["No Body"]
  print book

  let me = Person "Burak Orcun" 37
  print me
  print (getAge me)
