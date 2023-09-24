/**
 * @file main.cpp
 * @brief 
 * @author boo
 */

#include <iostream>
#include <osg/Geometry>
#include <osg/Geode>
#include <osgViewer/Viewer>

int main(int argc, char **argv) {
  osgViewer::Viewer viewer;
  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec3Array> normal = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
  osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;
  osg::ref_ptr<osg::Geode> root = new osg::Geode;
    
  vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
  vertices->push_back(osg::Vec3(1.0f, 0.0f, 0.0f));
  vertices->push_back(osg::Vec3(1.0f, 0.0f, 1.0f));
  vertices->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));

  vertices->push_back(osg::Vec3(0.0f, 2.0f, 0.0f));
  vertices->push_back(osg::Vec3(1.0f, 2.0f, 0.0f));
  vertices->push_back(osg::Vec3(1.0f, 2.0f, 1.0f));
  vertices->push_back(osg::Vec3(0.0f, 2.0f, 1.0f));

  normal->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));

  color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
  color->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
  color->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
  color->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));

  color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
  color->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
  color->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
  color->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));

  quad->setVertexArray(vertices);
  quad->setNormalArray(normal);
  quad->setNormalBinding(osg::Geometry::BIND_OVERALL);
  quad->setColorArray(color);
  quad->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

  quad->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, vertices->size()));

  root->addDrawable(quad.get());  

  viewer.setSceneData(root.get());
  
  return viewer.run();
}
