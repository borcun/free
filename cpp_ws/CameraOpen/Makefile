CXX = g++
CXXFLAGS = -Wall -c
SOURCES = opencamera.cpp
OBJ = opencamera.o
LIBS = -lopencv_highgui -lopencv_core
DEPS = opencv/highgui.h opencv/cxcore.h
EXECUTABLE = opencam

compile: ${SOURCES}
	$(CXX) $(CXXFLAGS) $(SOURCES)

all: ${OBJ}
	$(CXX) $(OBJ) -o $(EXECUTABLE) $(LIBS)

clean:
	rm -rf $(OBJ) $(EXECUTABLE)
