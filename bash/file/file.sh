#!/bin/bash

exec 4> output.txt

read -p "enter a word : " word
echo $word >&4

exec 4<&-
