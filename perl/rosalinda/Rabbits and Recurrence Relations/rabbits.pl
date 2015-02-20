#!/bin/perl

use warnings;

# recurrence relation function about rabbits
# general formula: f(n) = 3 * f(n-2) + f(n-1)
# initial step: f(0) = 0, f(1) = 1
sub rabbits {
	if($_[0] == 0) {
		return 0;
	}
	elsif($_[0] == 1) {
		return 1;
	}
	else {
		if($_[0] >= 2) {
			return rabbits($_[0] - 1, $_[1]) + $_[1] * rabbits($_[0] - 2, $_[1]);
		}
		return rabbits($_[0] - 1, $_[1]) + rabbits($_[0] - 2, $_[1]);
	}
}

$month = 33;
$litter = 3;

$res = rabbits $month, $litter;
print "result : $res\n";
