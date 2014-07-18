compile: list.c main.c
	gcc -Wall -c list.c main.c

all: list.o main.o
	gcc main.o list.o -o list

clear:
	rm -rf list.o main.o list