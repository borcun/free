& operator is used to match string. & refer source string in destination place.

echo abc | sed 's/abc/(abc)/'
echo abc | sed 's/abc/(&)/'
echo abc | sed 's/[a-z]*/(&)/'
echo abc | sed 's/[a-z]*/(& &)/'
echo abc | sed 's/[a-z]*/(& x &)/'
