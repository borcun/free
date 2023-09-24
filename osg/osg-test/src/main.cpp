#include <iostream>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>

int main(int argc, char **argv) {
  osg::ArgumentParser arguments(&argc, argv);
  std::string osg_file;
  osgViewer::Viewer viewer;

  if (3 != arguments.argc()) {
    std::cerr << "usage: ./osg-test --model <osg file>" << std::endl;
    return -1;
  }

  arguments.read("--model", osg_file);
  osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(osg_file);

  viewer.setSceneData(root.get());

  return viewer.run();
}
