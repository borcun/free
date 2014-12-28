#!/usr/bin/bash

#create readonly variable
readonly var="this variable is readonly"

echo "$var"
var="new string"
echo "$var"
