CC = gcc
DEPS = -lsqlite3

compile: insertion.c
	${CC} -Wall -c insertion.c

all: insertion.o
	${CC} insertion.o -o insertion ${DEPS}

clear:
	rm -rf insertion.o insertion staff.sqlite