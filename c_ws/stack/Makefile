CC = gcc

compile: Stack.c main.c
	${CC} -c *.c

all: Stack.o main.o
	${CC} *.o -o stack
	
clear:
	rm -rf *.o stack
