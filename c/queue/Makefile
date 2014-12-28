CC = gcc
CFLAGS = -Wall -c
SRC = main.c queue.c
OBJ = main.o queue.o
EXEC = queue

compile: ${SRC}
	${CC} ${CFLAGS} ${SRC}
	
all: ${OBJ}
	${CC} ${OBJ} -o ${EXEC}

clear:
	rm -rf ${OBJ} ${EXEC}
