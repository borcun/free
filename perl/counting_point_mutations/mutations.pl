#!/bin/perl

sub cmpDNA {
	$dna1 = @_[0];
	$dna2 = @_[1];
	$mismatch = 0;

	if(($len = length $dna1) != length $dna2) {
			print "DNA lengths must be same\n";
			return;
	}

	print "dna1: $dna1\ndna2: $dna2\n";
	
	for($i = 0 ; $i < $len-1 ; $i++) {
		# Perl uses 'ne' operator to compare strings
		if(substr($dna1, $i, 1) ne substr($dna2, $i, 1)) {
				$mismatch++;
		}
	}

	print "Mismatched : $mismatch\n";
}

cmpDNA "GAGCCTACTAACGGGAT", "CATCGTAATGACGGCCT";
