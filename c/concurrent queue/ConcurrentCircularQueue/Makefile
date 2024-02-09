CC=gcc
CFLAGS=-c -Wall
LFLAGS=-o
LIBS=-lpthread
SRC=cbuf.c main.c
OBJ=cbuf.o main.o
EXEC=cbuf

TEST_DEF=-DCBUF_TEST
TEST_SRC=${SRC} cbufTest.c
TEST_OBJ=${OBJ} cbufTest.o
TEST_EXEC=${EXEC}_test

RM=rm -rf

all:
	${CC} ${CFLAGS} ${SRC}
	${CC} ${OBJ} ${LFLAGS} ${EXEC} ${LIBS}

test:
	${CC} ${CFLAGS} ${TEST_DEF} ${TEST_SRC}
	${CC} ${TEST_OBJ} ${LFLAGS} ${TEST_EXEC} ${LIBS}

clean:
	${RM} ${OBJ} ${EXEC}

cleanTest:
	${RM} ${TEST_OBJ} ${TEST_EXEC}
