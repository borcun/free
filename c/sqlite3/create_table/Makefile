
CC = gcc
DEPS = -lsqlite3

compile: create_table.c
	${CC} -Wall -c create_table.c

all: create_table.o
	${CC} create_table.o -o create_table ${DEPS}

clear:
	rm -rf create_table.o staff.sqlite create_table
