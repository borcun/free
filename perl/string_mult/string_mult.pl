#!/bin/perl

print "enter a string: ";
chomp($str = <STDIN>);
print "enter how many repeat string you want to print: ";
$repeat = <STDIN>;

print $str x $repeat . "\n";
