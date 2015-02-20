#!/bin/perl -w

use warnings;

sub complement {
	my $data = $_[0];
	my $len = length $data;

	for($i=0 ; $i < $len ; $i++) {
		if("A" eq substr $data, $i, 1) {
			substr $data, $i, 1, "T";
		}
		elsif("C" eq substr $data, $i, 1) {
			substr $data, $i, 1, "G";
		}
		elsif("G" eq substr $data, $i, 1) {
			substr $data, $i, 1, "C";
		}
		elsif("T" eq substr $data, $i, 1) {
			substr $data, $i, 1, "A";
		}
		else {
			print "invalid nucleotide\n";
		}	
	}

	$data = reverse $data;
	print "$data\n";
}

my $filename = 'rosalind_revc.txt';
open(FH, "<$filename") or die "could not open $filename\n";
complement <FH>;
