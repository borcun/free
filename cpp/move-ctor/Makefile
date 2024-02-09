CXX=clang++

all:
	${CXX} -c -Wall -std=c++20 main.cpp
	${CXX} main.o -o main

debug:
	${CXX} -c -Wall -g -std=c++20 main.cpp
	${CXX} main.o -o main

clean:
	rm -rf main.o main 
