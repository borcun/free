#!/bin/perl

@names = ();

while (chomp($name = <STDIN>)) {
	push @names, $name;
}

foreach (@names) {
	print $_ . " ";
}

print "\n";
