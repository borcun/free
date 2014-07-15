CXX  = g++
LIBS = -L /opt/ros/hydro/lib/ -lroscpp -lrosconsole -lrostime -lroscpp_serialization
DEPS = talker.cpp listener.cpp
OBJ  = talker.o listener.o

compile: $(DEPS)
	${CXX} -Wall -c talker.cpp listener.cpp

link: $(OBJ)
	${CXX} listener.o -o listener ${LIBS}
	${CXX} talker.o -o talker ${LIBS}
	
clean:
	rm -rf talker.o listener.o talker listener
