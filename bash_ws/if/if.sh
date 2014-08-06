#!/usr/bin/bash

read -p "Enter first number  : "  num1
read -p "Enter second number : "  num2

if test $num1 -ge $num2
then
	echo "$num1 >= $num2"
else
	echo "$num1 < $num2"
fi
