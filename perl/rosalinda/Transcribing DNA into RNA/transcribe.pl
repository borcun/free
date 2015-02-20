#!/bin/perl -w

use warnings;

sub transcribe {
	my $data = $_[0];
	my $len = length $data;

	for($i=0 ; $i < $len ; $i++) {
		if("T" eq substr $data, $i, 1) {
			substr $data, $i, 1, "U";
		}
	}

	print "$data\n";
}

my $filename = 'rosalind_rna.txt';
open(FH, "<$filename") or die "could not open $filename\n";
transcribe <FH>;
