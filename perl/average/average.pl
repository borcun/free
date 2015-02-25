#!/bin/perl -w

use warnings;
use strict;
use 5.018;

sub average {
	my (@list) = @_;
	my $size = @_;
	my $sum = 0;

	foreach (@list) {
		$sum += $_;
	}
	
	$sum / $size;
}

my @list = (10, 23, 42, 75, 88);
my $res = average (@list);
print "result: $res\n";
