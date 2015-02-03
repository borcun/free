#!/opt/local/bin/perl

# this line is comment line. the sharp operator is used for one line comment.

=begin
		These lines are comment lines as well. the assignment and begin keyword
		starts multi-line comment lines, assignment and cut keyword ends comment
		lines.

		In Perl, function calls are made both using parenthesis or not. 
		Both calls are valid. Besides, Perl file extension is .pl or .PL.

		The whitespace character is ignored but newline characters is processed.

		The first line is shebang which indicates perl binary file path, so
		if the file execution permission is set, it can be executed with ./.
=cut

$var = << "EOF";
this lines are documentation lines which is created with EOF keyword by
assign to a variable. then, when the variable is printed, the documentation
is printed.
EOF
print "$var\n";

print "Hello World without parenthesis\n";
print("Hello World with parenthesis\n");
print    "Hello World with whitespace\n";
print "Hello
          World with new line\n";
