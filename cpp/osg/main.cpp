#include <iostream>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>

using namespace osg;
using namespace osgViewer;
using namespace osgDB;

int main()
{
	osgViewer::Viewer viewer;
	
	viewer.setSceneData( osgDB::readNodeFile("model.obj") );

	return viewer.run();
}