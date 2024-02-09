SDIR=src
CXX=g++
CFLAGS=-c -Wall -std=c++20
LFLAGS=-o
DFLAGS=-g
SRC=$(SDIR)/message.cpp $(SDIR)/messageFactory.cpp $(SDIR)/subscriber.cpp $(SDIR)/publisher.cpp $(SDIR)/messageService.cpp main.cpp
OBJ=message.o messageFactory.o subscriber.o publisher.o messageService.o main.o
EXEC=main
INC=-I inc
RM=rm -rf

all:
	$(CXX) $(CFLAGS) $(SRC) $(INC)
	$(CXX) $(OBJ) $(LFLAGS) $(EXEC)

debug:
	$(CXX) $(CFLAGS) $(DFLAGS) $(SRC) $(INC)
	$(CXX) $(OBJ) $(LFLAGS) $(EXEC)

clean:
	$(RM) $(OBJ) $(EXEC)

