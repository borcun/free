#include "ABRobotScanner.hpp"

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	ABRobotScanner w;

	w.show();

	return a.exec();
}
