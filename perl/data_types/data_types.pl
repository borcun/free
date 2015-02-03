#!/opt/local/bin/perl

# this file is about data types in Perl, 3 data types: scalar ($), array(@), hash(%).

=begin
		# scalar data type is simple data type for one variable such as integer, float,
		  character, hexadecimal, octal. scalar data type starts with $ sign.
		# array data type is ordered scalar list and starts with @ sign. On contarty
		  standart, Perl use negative index for array like Python. In addition, the
		  sequential number arrays are available in Perl.
		# last data type is hash data type which occurs keys and values as known map.
		  it stores pairs and starts with % sign.

		In Perl, there is no type-checking or any constraint in order to create variable.
=cut

print "------------------------------------------------------------------------\n";
$name = "Orcun";
$age = 28;
$salary = 14.58;

print "Scalar Data Types\n=================\n";
print "Employee \'$name\', age $age, earns $salary in a week.\n\n";

# there is no restriction about each array element types, they can be different
# scalar types.

@arr = ("Orcun", 28, 14.58);
$size = @arr;

print "Array Data Types\n===============\n";
print "Employee \'$arr[0]\', age $arr[1], earns $arr[2] in a week.\n";
print "Print all array elements[$size] : @arr\n\n";

%map =("name", "Orcun", "age", 28, "salary", 14.58);

# the hash data type is flexible as array, all keys or values doesn't have to be same
# type.

print "Hash Data Types\n==============\n";
print "Employee \'$map{'name'}\', age $map{'age'}, earns $map{'salary'} in a week.\n";
print "------------------------------------------------------------------------\n\n";

print "------------------------------------------------------------------------\n";
@arr2 = (1..10);
print "Sequential Number Array\n=======================\n";
print "\@arr2 = (1..10) = @arr2\n";
print "\$#arr2 gives arr last index: $#arr2\n";
print "------------------------------------------------------------------------\n\n";

print "------------------------------------------------------------------------\n";
print "# push and pop functions perform on array and treat it as stack\n";

push(@arr2, 12);
print "after push(\@arr2, 12) : @arr2\n";

pop(@arr2);
print "after pop(\@arr2) : @arr2\n";

pop(@arr2);
print "after pop(\@arr2) : @arr2\n\n";

print "# we can get a slice from array with .. operator or ordered indices.\n";
print "\@arr2[3..6] : @arr2[3..6]\n";
print "\@arr2[3,4,5] : @arr2[3,4,5]\n";
print "------------------------------------------------------------------------\n\n";

$comment = << "EOF";
------------------------------------------------------------------------
# split function is used to split array with a regex

# join function performs vice versa, it creates a string which
# contains a regex.

# sort function sorts array with using a optional sort function.
# by default, it is numeric comparison function which uses ASCII values.
EOF

print "$comment\n";
$str = "asm-lisp-c-c++-perl-java-python";
@arr3 = split('-', $str);
print "string: $str\n";
print "\@arr3 = split('-', \$str): @arr3\n";
@arr3 = sort(@arr3);
print "sort(\@arr3) : @arr3\n";
$str = join('.', @arr3);
print "\$str = join('.', \@arr3): $str\n";
print "------------------------------------------------------------------------\n\n";

print "------------------------------------------------------------------------\n";
$comment = << "EOF";
 # hash data types are create in manners more than one. 
 # first of them, we used above. 
 # the second one is like PHP hash
 # the last one is specific to Perl. It uses hyphen for keys.

  %map1 = ('key1', value1, 'key2', value2);
  %map2 = ('key1' => value1, 'key2' => value2);  // in PHP 
  %map3 = (-key1 => value1, -key2 => value2); // with hyphen, no quotation
EOF

print "$comment\n";

%map1 = (-key1 => 10, -key2 => 20, -key3 => 30, -key4 => 40);

print "# The slice operation is performed in hash data types as well.\n";
print "\@map1{\-key2, \-key3} : @map1{-key2, -key3}\n";

print "------------------------------------------------------------------------\n\n";
