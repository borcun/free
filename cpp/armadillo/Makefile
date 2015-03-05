compile: arma.cpp
	g++ -c -Wall arma.cpp
	
link: arma.o
	g++ arma.o -o arma -larmadillo
	
all:
	g++ -c -Wall arma.cpp
	g++ arma.o -o arma -larmadillo

clear:
	rm -rf arma.o arma
