CC=g++
CFLAGS=-c -Wall
LIB= -lsfml-system -lsfml-window -lsfml-graphics -L /usr/lib/x86_64-linux-gnu/
SRC=open_window.cpp
OBJ=open_window.o
EXEC=open_window
RM=rm -rf

all:
	${CC} ${CFLAGS} ${SRC}
	${CC} ${OBJ} -o ${EXEC} ${LIB}

clean:
	${RM} ${OBJ} ${EXEC}
