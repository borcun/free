CC = gcc
SRC = udp_attack.c
OBJ = udp_attack.o
EXE = udp_attack
RM = rm -rf

all: ${SRC}
	${CC} -c -Wall ${SRC}
	${CC} ${OBJ} -o ${EXE}

compile: ${SRC}
	${CC} -c -Wall ${SRC}

link: ${OBJ}
	${CC} ${OBJ} -o ${EXE}

clean: ${OBJ} ${EXE}
	${RM} ${OBJ} ${EXE}
