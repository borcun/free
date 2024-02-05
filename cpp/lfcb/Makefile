CXX=g++
CFLAGS=-c -Wall
LFLAGS=-o
DFLAGS=-g
SRC=lfcb.cpp main.cpp
OBJ=lfcb.o main.o
LIBS=-lpthread
EXEC=lfcb
TEST_SRC=lfcb.cpp test.cpp
TEST_OBJ=lfcb.o test.o
TEST_EXEC=lfcb_test
RM=rm -rf

all:
	${CXX} ${CFLAGS} ${SRC}
	${CXX} ${OBJ} ${LFLAGS} ${EXEC} ${LIBS}

debug:
	${CXX} ${CFLAGS} ${DFALGS} ${SRC}
	${CXX} ${OBJ} ${LFLAGS} ${EXEC} ${LIBS}

test:
	${CXX} ${CFLAGS} ${TEST_SRC}
	${CXX} ${TEST_OBJ} ${LFLAGS} ${TEST_EXEC}	

clean:
	${RM} ${OBJ} ${EXEC}

clean_test:
	${RM} ${TEST_OBJ} ${TEST_EXEC}
