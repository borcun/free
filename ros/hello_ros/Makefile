CXX  = g++
LIBS = -L /opt/ros/hydro/lib/ -lroscpp -lrosconsole -lrostime
DEPS = hello_ros.cpp
OBJ  = hello_ros.o 

compile: $(DEPS)
	${CXX} -Wall -c hello_ros.cpp

link: $(OBJ)
	${CXX} hello_ros.o -o hello_ros ${LIBS}
	
clean:
	rm -rf hello_ros.o hello_ros
