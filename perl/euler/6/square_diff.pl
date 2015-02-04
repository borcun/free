#!/bin/perl

$LIMIT = 100;
$square_of_sum = 0;
$sum_of_square = 0;

for($i=1 ; $i <= $LIMIT ; $i++) {
		$square_of_sum += $i;
		$sum_of_square += ($i ** 2);
}

$square_of_sum **= 2;

print "square of sum : $square_of_sum\n";
print "sum of square : $sum_of_square\n";
print "diff : " . ($square_of_sum - $sum_of_square) . "\n";


