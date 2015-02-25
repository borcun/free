#!/bin/perl -w

use warnings;
use strict;
use 5.018;

sub total {
	my (@list) = @_;
	my $sum = 0;
	
	foreach (@list) {
		$sum += $_;
	}
	
	return $sum;
}

my @list = (1, 3, 4, 2, 6, 5);
my $res = total(@list);
print "result: $res\n";
