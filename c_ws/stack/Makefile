CC = gcc
CFLAGS = -c -Wall
SRC = Stack.c main.c
OBJ = Stack.o main.o
EXEC = stack

compile: ${SRC}
	${CC} ${CFLAGS} ${SRC}

all: ${OBJ}
	${CC} ${OBJ} -o ${EXEC}

clear:
	rm -rf ${OBJ} ${EXEC}
