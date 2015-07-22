#!/bin/bash

exec 4< output.txt
cat <&4
exec 4<&-
