#!/bin/perl -w

use warnings;
use strict;
use 5.018;

my @list = ();

sub greet {
	my $name = $_[0];

	if (0 == @list) {
		print ("Hello, $name! You\'re first person here\n");
	}
	else {
		print ("Hi, $name, ");
		
 		foreach (@list) {
			print ("$_ ");
		}
		
		print ("have seen you\n");
	}
	
	push (@list, $name);
}

greet ("Burak");
greet ("Orcun");
greet ("Ahmet");
