build :
	g++ -c main.cpp ABVisualModel.cpp ABCalibration.cpp -I /usr/local/include/
	g++ main.o ABVisualModel.o ABCalibration.o -o run -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -losg -losgViewer -losgGA -losgDB

compile :
	g++ -c main.cpp ABVisualModel.cpp ABCalibration.cpp -I /usr/local/include/

link : 
	g++ main.o ABVisualModel.o ABCalibration.o -o run -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -losg -losgViewer -losgGA -losgDB
	
run : 
	./run
