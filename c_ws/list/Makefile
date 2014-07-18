<<<<<<< HEAD
compile: list.c main.c
	gcc -Wall -c list.c main.c

all: list.o main.o
	gcc main.o list.o -o list

clear:
	rm -rf list.o main.o list
=======
CC = gcc
FLAGS = -Wall -c

compile: list.c main.c
	${CC} ${FLAGS} main.c list.c

all: list.o main.o
	${CC} list.o main.o -o list

clear:
	rm -rf list.o main.o list
>>>>>>> 62df991aa43b1193490e7f5ac14fd1f1d6a5e78f
