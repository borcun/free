#!/bin/perl

sub prime
{
	$n = @_[0]; # get first paramater
		
	for($i=2 ; $i <= $n / 2 ; $i++) {
		if($n % $i == 0) {
			return -1;
		}
	}

	return 0;
}

$LIMIT = 20;
$num = 1;

for($j=2 ; $j < $LIMIT ; $j++) {
	if(0 == prime($j)) {
		print "j : $j\n";
		$num *= $j;
	}
}

print "number: $num\n";
