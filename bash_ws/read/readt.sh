#!/usr/bin/bash

read -t 3 -p "Enter name: " name

if test "$name" = ""
then
    printf "\ntimeout\n"
else
    printf "Name: %s\n" $name
fi
