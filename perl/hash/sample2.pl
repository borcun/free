#!/bin/perl -w

use strict;
use warnings;
use 5.0.18;

my %table;
my $flag = 1;

while( $flag ) {
	print "enter a name: ";
	my $name = <STDIN>;
	
	if( $table{ $name } ) {
		print "this name was already in the table\n";
		chomp( $name );
		$table{ $name } += 1;
	}
	else {
		print "this name is new in the table\n";
		chomp( $name );
		$table{ $name } = 1;
	}

	print "enter for continue, ctrl+D for quit : ";
	$flag = <STDIN>;
}

print "\nHash Table\n----------\n":

foreach( keys( %table ) ) {
	print "$_ $table{ $_ }\n";
}
