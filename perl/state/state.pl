#!/bin/perl -w

use strict;
use 5.018;

sub myPrint {
	state $x = 1;
	$x++;
	print "x: $x\n";
}

print "first call, ";
myPrint();
print "second call, ";
myPrint();
print "third call, ";
myPrint();
