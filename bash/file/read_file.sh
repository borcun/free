#!/bin/bash

# open output.txt to read and assign its file descriptor as 4
exec 4< output.txt
# print whole content of file
cat <& 4
# close file descriptor
exec 4<&-
