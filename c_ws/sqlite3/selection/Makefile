CC = gcc
CFLAGS = -Wall -c
SRC = selection.c
OBJ = selection.o
EXEC = selection
DEPS = -lsqlite3

compile: ${SRC}
	${CC} ${CFLAGS} ${SRC}

all: ${OBJ}
	${CC} ${OBJ} -o ${EXEC} ${DEPS}

clear:
	rm -rf ${OBJ} ${EXEC}