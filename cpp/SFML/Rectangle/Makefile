CC=g++
CFLAGS=-c -Wall
LIB=-lsfml-system -lsfml-graphics -lsfml-window -L /usr/lib/x86_64-linux-gnu/
SRC=rect.cpp
OBJ=rect.o
EXEC=rect
RM=rm -rf

all:
	${CC} ${CFLAGS} ${SRC}
	${CC} ${OBJ} -o ${EXEC} ${LIB}

clean:
	${RM} ${OBJ} ${EXEC}
