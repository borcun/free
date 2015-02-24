#!/bin/perl

# the programs gets input from user and reverse the whole input.
# Note: it can be terminated with Ctrl+D combination.

$sen = "";
$line;

while(chomp($line = <STDIN>)) {
	$sen .= $line;
}

print $sen . "\n";
$sen = reverse $sen;
print $sen . "\n";
