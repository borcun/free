CC=g++
CFLAGS=-c -Wall
LFLAGS=-o
DBGFLAGS=-c -Wall -g
LIBS=-lm
SRC=point.cpp convex_hull.cpp main.cpp
OBJ=point.o convex_hull.o main.o
EXEC=convex_hull
RM=rm -rf

all:
	${CC} ${CFLAGS} ${SRC}
	${CC} ${OBJ} ${LFLAGS} ${EXEC} ${LIBS}

debug:
	${CC} ${DBGFLAGS} ${SRC}
	${CC} ${OBJ} ${LFLAGS} ${EXEC} ${LIBS}

clean:
	${RM} ${OBJ} ${EXEC}
