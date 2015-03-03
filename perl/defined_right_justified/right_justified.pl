#!/bin/perl

@content = ();

while( <STDIN> ) {
	push( @content, $_ );
}

foreach( @content ) {
	printf( "%20s", $_ );
}
