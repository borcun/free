CXX = g++
CXXFLAGS = -c -Wall
SOURCES = opencamera.cpp
OBJ = opencamera.o
LIBS = -lopencv_highgui -lopencv_core
DEPS = opencv/highgui.h opencv/cxcore.h
EXECUTABLE = opencam

opencam:
	$(CXX) $(CXXFLAGS) $(SOURCES)
	$(CXX) $(OBJ) -o $(EXECUTABLE) $(LIBS)

clean :
	rm -rf $(OBJ) $(EXECUTABLE)
