CC = gcc
CFLAGS = -Wall -c
SRC = main.c list.c
OBJ = main.o list.o
EXEC = list

compile: ${SRC}
	${CC} ${CFLAGS} ${SRC}

all: ${OBJ}
	${CC} ${OBJ} -o ${EXEC}

clear:
	rm -rf ${OBJ} ${EXEC}
