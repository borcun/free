#!/bin/perl

sub prime10001st
{
  $n = 4;
	$counter = 2; # for 2 and 3

	while($counter != 10001) {
   	$true = 0;	  
   
		for($i=2 ; $i <= $n / 2 ; $i++) {
		  if($n % $i == 0) {
			  $true = 0;
			  last;
		  }
			else {
				$true = 1;
			}
	  } # end of for

    if($true == 1) {
  		print "$n [$counter]\n";
			$counter++;
		}

		$n++;
	}

  # ignore last unnecessary increasing
	print "10001st number: " . $n-- . "\n";
}

prime10001st();
