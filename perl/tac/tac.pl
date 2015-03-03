#!/bin/perl -w

# the program treats like tac command.

use warnings;
use 5.0.18;

$ARGC = @ARGV;

if( 0 == $ARGC ) {
	print "usage: ./tac.pl <parameters>\n";
	exit 0;
}

@content = ();

for ($i=0 ; $filename = $ARGV[$i]; $i++ ) {
	if ( !open( FILE, "<$filename") ) {
		die "$! [$filename]";
	}
	
	while ( <FILE> ) {
		chomp;
		unshift( @content, $_ );
		unshift( @content, "\n" );
	}
}

print @content; 
print "\n";
