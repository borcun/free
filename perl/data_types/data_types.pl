#!/opt/local/bin/perl

# this file is about data types in Perl, 3 data types: scalar ($), array(@), hash(%).

=begin
		# scalar data type is simple data type for one variable such as integer, float,
		  character, hexadecimal, octal. scalar data type starts with $ sign.
		# array data type is ordered scalar list and starts with @ sign. 
		# last data type is hash data type which occurs keys and values as known map.
		  it stores pairs and starts with % sign.

		In Perl, there is no type-checking or any constraint in order to create variable.
=cut

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
