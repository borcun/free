#!/opt/local/bin/perl

$var = << "EOF";
===========================================================================
brief  : the program finds the largest prime factor.
author : boo
date   : Feb 02, 2015
============================================================================
EOF

print $var . "\n";

$num = 600851475143;
$count = 2;
$init = 2;

while ($num != 1) {
  if($num % $count == 0) {
	  print "$count\n";
		$num /= $count;
		$count = $init;
	}
	else {
	  $count++;
  }
}
