#!/bin/perl -w

use warnings;
use strict;

$_ = "hello there using perl";

if( /there/ ) {
	print "\'$_\' sentence contains \'there\' word\n";
} else {
	print "\'$_\' sentence doesn't contain \'there\' word\n";
}
