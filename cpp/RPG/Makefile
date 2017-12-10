CC=g++
CFLAGS=-c -Wall
LFLAGS=-o
CLEAN=rm -rf
SRC=GameObject.cpp FinishPoint.cpp Gold.cpp Character.cpp Player.cpp Enemy.cpp GameMap.cpp main.cpp
OBJ=GameObject.o FinishPoint.o Gold.o Character.o Player.o Enemy.o GameMap.o main.o
EXEC=rpg

all:
	${CC} ${CFLAGS} ${SRC}
	${CC} ${OBJ} ${LFLAGS} ${EXEC}

clean:
	${CLEAN} ${OBJ} ${EXEC}
