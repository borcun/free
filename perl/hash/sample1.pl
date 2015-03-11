#!/bin/perl -w

use strict;
use warnings;
use 5.0.18;

my %names;
my $flag = 1;

while( $flag ) {
	print "enter name: ";
	my $name = <STDIN>;
	chomp( $name );
	print "enter surname: ";
	my $surname = <STDIN>;
	chomp( $surname );
	$names{ $name } = $surname;

	print "enter for continue, ctrl+D for quit";
	$flag = <STDIN>;
}

print "\nHash Table\n-----------\n";

foreach( keys( %names ) ) {
	print "$_ $names{ $_ }\n";
}
