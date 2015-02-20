#!/bin/perl -w

use warnings;

# function that counts GC content and returns percent of GC content
sub countGC 
{
	$GC = $_[0];
	# ignore new line character by substracting 1
	$len = length($GC) - 1;
	$count = 0;

	for($i=0 ; $i < $len ; $i++) {
		if("C" eq substr($GC, $i, 1) or "G" eq substr($GC, $i, 1)) {
			$count++;
		}
	}

	# return percent
	return $count * 100 / $len;
}

$max_gc_count = 0.0;
$max_gc_name = "";
$temp_gc_name = "";
$temp_gc_content = "";

$filename = "rosalind_gc.txt";
open(FH, "<$filename") or die("could not open $filename\n");

while(<FH>) {
	if(">" eq substr($_, 0, 1)) {
		if(length($temp_gc_content) == 0) {
			$temp_gc_count = countGC($temp_gc_content);

			if($temp_gc_count > $max_gc_count) {
				$max_gc_count = $temp_gc_count;
				$max_gc_name = $temp_gc_name;
			}

			print $temp_gc_name . " [$temp_gc_count]\n";
			$temp_gc_content = "";
			print "temp gc content: $temp_gc_content [" . length($temp_gc_content) . "]";
		}
		else {
			$temp_gc_name = $_;
		}
	}
	else {
		$temp_gc_content .= $_;
	}
}

$temp_gc_count = countGC($temp_gc_content);

if($temp_gc_count > $max_gc_count) {
	$max_gc_count = $temp_gc_count;
	$max_gc_name = $temp_gc_name;
}

print $temp_gc_name . " [$temp_gc_count]\n";
$temp_gc_content = "";


print "\nmax_gc_name  : $max_gc_name";
print "max_gc_count : $max_gc_count\n";

