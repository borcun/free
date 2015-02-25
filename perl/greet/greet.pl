#!/bin/perl -w

use warnings;
use strict;
use 5.018;

my @list = ();

sub greet {
	my $name = $_[0];
	
	print "Hello, $name! You\'re welcome.";
	
	foreach (@list) {
		print "\n$_ is also here.";
	}
	
	print "\n";
	push (@list, $name);
}

greet ("Burak");
greet ("Orcun");
