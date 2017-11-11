/*
 * Description : ControllerScene Class
 * Author	   : Burak Orcun OZKABLAN
 * Date		   : 14.06.2012
 */

#ifndef CONTROLLERSCENE_H
#define CONTROLLERSCENE_H

#include <iostream>
#include <string.h>
#include <osg/Node>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Vec3>
#include <osgDB/ReadFile>
#include <osgViewer/View>
#include <osgText/Text>
#include <osgText/Font>

#define MAX_CHAR 256
#define CS_X_POS 1150
#define CS_Y_POS 80

using namespace std;
using namespace osg;
using namespace osgDB;
using namespace osgText;
using namespace osgViewer;

// ControllerScene Class
class ControllerScene : public osgViewer::View
{
public:
	// constructors
	ControllerScene();
	ControllerScene(const char *name, const short width, const short height);
	//// destructor
	virtual ~ControllerScene();

	// set and get functions of scene name
	void setControllerSceneName(const char *name);
	char* getControllerSceneName() const;
	
	// set and get functions of scene width
	void setControllerSceneWidth(const short width);
	short getControllerSceneWidth() const;
	
	// set and get functions of scene height
	void setControllerSceneHeight(const short height);
	short getControllerSceneHeight() const;

private:
	char *sceneName;
	short sceneWidth;
	short sceneHeight;

	// geode object
	osg::ref_ptr<osg::Geode> geode;

	// ro sphere and line
	osg::ref_ptr<osg::Geometry> roGeomerty;
	osg::ref_ptr<osg::Vec3Array> roLine;
	osg::ref_ptr<osg::ShapeDrawable> roSphere;
	osg::ref_ptr<osg::Vec4Array> roColor;
	// teta sphere and line
	osg::ref_ptr<osg::Geometry> tetaGeomerty;
	osg::ref_ptr<osg::Vec3Array> tetaLine;
	osg::ref_ptr<osg::ShapeDrawable> tetaSphere;
	osg::ref_ptr<osg::Vec4Array> tetaColor;
	// fi sphere and line	
	osg::ref_ptr<osg::Geometry> fiGeomerty;
	osg::ref_ptr<osg::Vec3Array> fiLine;
	osg::ref_ptr<osg::ShapeDrawable> fiSphere;
	osg::ref_ptr<osg::Vec4Array> fiColor;

	// function that initialize attributes to create screen
	void init(const char *, const short, const short);
};

#endif