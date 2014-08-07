#!/usr/bin/bash
str1="burak orcun ozkablan"

read -r name1 name2 surname <<<"$str1"
printf "%s\n%s\n%s\n" $name1 $name2 $surname

# change IFS
str2="burak:orcun:ozkablan"
old="$IFS"
IFS=:

read -r name1 name2 surname <<<"$str2"
printf "\n%s\n%s\n%s\n" $name1 $name2 $surname

IFS="$old"
