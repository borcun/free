#!/bin/perl -w

use strict;
use warnings;
use 5.0.18;

foreach( keys( %ENV ) ) {
		print "$ENV{ $_ }\n";
}
