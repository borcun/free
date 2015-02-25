#!/bin/perl -w

use warnings;
use strict;
use 5.018;

sub total {
	my $sum = 0;
	
	foreach (1..1000) {
		$sum += $_;
	}
	
	return $sum;
}

my $res = total();
print "result: $res\n";
