CC = gcc
DEPS = -lsqlite3

compile: connection.c
	${CC} -Wall -c connection.c
	
all: connection.o
	${CC} connection.o -o connection ${DEPS}
	
clear:
	rm -rf connection.o connection.db connection
