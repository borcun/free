#!/bin/bash

# open output.txt to read and assign its file descriptor as 4
exec 4< output.txt
# read a word to variable
read -u 4 word
echo $word
# close file descriptor
exec 4<&-
