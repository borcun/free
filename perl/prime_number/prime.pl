#!/bin/perl

sub prime
{
		$num = @_[0]; # get first paramater
		
		for($i=2 ; $i < $num / 2 ; $i++) {
				if($num % $i == 0) {
						return -1;
				}
		}

		return 0;
}

print "Enter a number: ";
$var = <STDIN>;

$res = prime($var);
print "res : $res\n";
