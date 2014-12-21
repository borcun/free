() and \1 operators are special usage operators.

[a-z]* means that all words start from any character between a and z.
\([a-z]*\) parses abcd from abcd1234 world.
finally, .* splits abcd and \1 refers to abcd only.

echo abcd123 | sed 's/\([a-z]*\)/\1/'
echo abcd123 | sed 's/\([a-z]*\).*/\1/'
