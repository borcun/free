CC = gcc
FLAGS = -Wall -c

compile: list.c main.c
	${CC} ${FLAGS} main.c list.c

all: list.o main.o
	${CC} list.o main.o -o list

clear:
	rm -rf list.o main.o list
