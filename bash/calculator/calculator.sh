#!/bin/bash
read -p "enter an op : " op; read -p "enter two numbers : " x y; echo "$x $op $y = $(( $x $op $y ))"