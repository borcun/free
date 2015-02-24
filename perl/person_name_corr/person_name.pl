#!/bin/perl

@numbers = (1..5);
@names = qw(burak orcun ozkablan perl oreilly);
@inputs = ();

while (<STDIN>) {
	push(@input, $_);
}

foreach (@input) {
	print @names[$_ - 1] . "\n";
}
