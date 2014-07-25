CC = gcc
CFLAGS = -Wall -c
DEPS = -lpthread
SRC = timing.c
OBJ = timing.o
EXEC = timing

compile: ${SRC}
	${CC} ${CFLAGS} ${SRC}

all: ${OBJ}
	${CC} ${OBJ} -o ${EXEC} ${DEPS}

clear:
	rm -rf ${OBJ} ${EXEC}