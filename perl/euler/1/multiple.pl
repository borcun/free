#!/opt/local/bin/perl

$var = << "EOF";
===========================================================================
brief  : this program finds sum of all the multiplies of 3 and 5 below 1000
author : boo
date   : Feb 02, 2015
============================================================================
EOF

print $var . "\n";

$LIMIT = 1000;
$sum = 0;

for($i=0 ; $i < $LIMIT ; $i++) {
		if($i % 3 == 0 || $i % 5 == 0) {
				$sum += $i; 
		}
}

print "multiple of three : $sum\n";
