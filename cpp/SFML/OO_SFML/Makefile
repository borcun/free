CC=g++
CFLAGS=-c -Wall
LIB=-lsfml-system -lsfml-graphics -lsfml-window -L /usr/lib/x86_64-linux-gnu/
SRC=TestWindow.cpp main.cpp
OBJ=TestWindow.o main.o
EXEC=test_window
RM=rm -rf

all:
	${CC} ${CFLAGS} ${SRC}
	${CC} ${OBJ} -o ${EXEC} ${LIB}

clean:
	${RM} ${OBJ} ${EXEC}
