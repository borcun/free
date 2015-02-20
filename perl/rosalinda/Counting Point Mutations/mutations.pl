#!/bin/perl

sub cmpDNA {
	$s = @_[0];
	$t = @_[1];
	$dh = 0;

	if(($len = length $s) != length $t) {
			print "DNA lengths must be same\n";
			return;
	}

	print "s: $s\nt: $t\n";
	
	for($i = 0 ; $i < $len-1 ; $i++) {
		# Perl uses 'ne' operator to compare strings
		if(substr($s, $i, 1) ne substr($t, $i, 1)) {
				$dh++;
		}
	}

	print "dh : $dh\n";
}

cmpDNA "GAGCCTACTAACGGGAT", "CATCGTAATGACGGCCT";
