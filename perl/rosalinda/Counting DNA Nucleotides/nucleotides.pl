#!/bin/perl -w

sub countNucleotides {
	my $data = $_[0];
	my $len = length $data;
	my $A = 0;
	my $C = 0;
	my $G = 0;
	my $T = 0;

	for($i = 0 ; $i < $len-1 ; $i++) {
		if("A" eq substr $data, $i, 1) {
			$A++;
		}
		elsif("C" eq substr $data, $i, 1) {
			$C++;
		}
		elsif("G" eq substr $data, $i, 1) {
			$G++;
		}
		elsif("T" eq substr $data, $i, 1) {
			$T++;
		}
		else {
			print "invalid nucleotide\n";
		}
	} # end of for

	print "$A $C $G $T\n";
}

my $filename = 'rosalind_dna.txt';
open(FH, "<$filename") or die "could not open $filename\n";
my $content = <FH>;
countNucleotides $content;
