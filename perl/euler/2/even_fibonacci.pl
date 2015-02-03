#!/opt/local/bin/perl

$var = << "EOF";
===========================================================================
brief  : the program finds sum of even numbers in fibonnaci series which 
		     does not exceeded four million.
author : boo
date   : Feb 02, 2015
============================================================================
EOF

print $var . "\n";

$prev = 1;
$next = 1;
$LIMIT = 4000000;
$sum = 0;

while($next < $LIMIT) {
		if($next % 2 == 0) {
				$sum += $next;
		}

		$temp = $next;
		$next += $prev;
		$prev = $temp;
}

printf "total number: $sum\n";

