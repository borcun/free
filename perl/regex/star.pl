#!/bin/perl -w

use warnings;
use strict;

$_ = "hello there using perl";

if( /the*/ ) {
	print "\'$_\' sentence contains words start with \'the\'\n";
} 
else {
  print "\'$_\' sentence doesn't contain words start with \'the\'\n";
}
