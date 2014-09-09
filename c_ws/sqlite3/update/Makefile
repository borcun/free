CC = gcc
DEPS = -lsqlite3

compile: update.c
	${CC} -Wall -c update.c

all: update.o
	${CC} update.o -o update ${DEPS}

clean:
	rm -rf update.o update staff.sqlite