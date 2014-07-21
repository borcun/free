CC = gcc
CFLAGS = -Wall -c
SRC = polynom.c
OBJ = polynom.o
EXEC = polynom

compile: ${SRC}
	${CC} ${CFLAGS} ${SRC}

all: ${OBJ}
	${CC} ${OBJ} -o ${EXEC}

clear:
	rm -rf ${OBJ} ${EXEC}
