#!/usr/bin/python

temp_num = int(raw_input("Enter a number: "))
inverse_num = 0
real_num = temp_num

while temp_num > 0:
	inverse_num *= 10
	inverse_num += temp_num % 10
	temp_num /= 10

if inverse_num == real_num:
	print "The number is palindrome"
else:
	print "The number is not palindrome"

