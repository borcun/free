/**
 * @file main.cpp
 * @brief The osg::Geode (geometry node) class correspondes the the leaf node of a scene graph.
 *        It has no child nodes, but always contains geometry information for rendering.
 *        The geometry data to be drawn are stored in a set of osg::Drawable objects managed by
 *        by osg::Geode.
 *        OSG provides an osg::ShapeDrawable class inherited from the osg::Drawable base class.
 *        It always includes an osg::Shape object to indicate the specified geometry's type and
 *        properties.
 * @author boo
 */

#include <iostream>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgViewer/Viewer>

int main(int argc, char **argv) {
  osgViewer::Viewer viewer;
  osg::ref_ptr<osg::ShapeDrawable> box = new osg::ShapeDrawable();
  osg::ref_ptr<osg::ShapeDrawable> sphere = new osg::ShapeDrawable();
  osg::ref_ptr<osg::ShapeDrawable> cone = new osg::ShapeDrawable();
  osg::ref_ptr<osg::ShapeDrawable> capsule = new osg::ShapeDrawable();
  osg::ref_ptr<osg::ShapeDrawable> cylinder = new osg::ShapeDrawable();
  osg::ref_ptr<osg::Geode> root = new osg::Geode();

  box->setShape(new osg::Box(osg::Vec3(-6.0f, 0.0f, 0.0f), 2.0f, 2.0f, 1.0f));
  sphere->setShape(new osg::Sphere(osg::Vec3(-3.0f, 0.0f, 0.0f), 1.0f));
  sphere->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
  cone->setShape(new osg::Cone(osg::Vec3(0.0f, 0.0f, 0.0f), 1.0f, 1.0f));
  cone->setColor(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
  capsule->setShape(new osg::Capsule(osg::Vec3(3.0f, 0.0f, 0.0f), 1.0f, 1.0f));
  capsule->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
  cylinder->setShape(new osg::Cylinder(osg::Vec3(6.0f, 0.0f, 0.0f), 1.0f, 1.0f));
  cylinder->setColor(osg::Vec4(0.0f, 1.0f, 1.0f, 1.0f));
  
  root->addDrawable(box.get());
  root->addDrawable(sphere.get());
  root->addDrawable(cone.get());
  root->addDrawable(capsule.get());
  root->addDrawable(cylinder.get());

  viewer.setSceneData(root.get());
  
  return viewer.run();
}
