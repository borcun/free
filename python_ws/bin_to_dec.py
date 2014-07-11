#!/usr/bin/python

bin = int(raw_input("Enter a binary number which starts 1: "))
dec = 0
i = 0

real_bin = bin

while bin > 0:
	dec += (2 ** i) * (bin % 10)
	i += 1
	bin /= 10

print "binary [{0}] == decimal [{1}]".format(real_bin, dec)
