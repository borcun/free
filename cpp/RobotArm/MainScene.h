/*
 * Description : MainScene Class
 * Author	   : Burak Orcun OZKABLAN
 * Date		   : 14.06.2012
 */

#ifndef MAINSCENE_H
#define MAINSCENE_H

#include <iostream>
#include <string.h>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgViewer/View>
#include <osg/MatrixTransform>
#include <osg/PolygonMode>
#include "Utility.h"

#define MAX_CHAR 256
#define MS_X_POS 120
#define MS_Y_POS 80

using namespace std;
using namespace osg;
using namespace osgDB;
using namespace osgViewer;

// MainScene Class
class MainScene : public osgViewer::View
{
public:
	// constructors
	MainScene();
	MainScene(const char *name, const short width, const short height);
	//// destructor
	virtual ~MainScene();

	// set and get functions of scene name
	void setMainSceneName(const char *name);
	char* getMainSceneName() const;
	
	// set and get functions of scene width
	void setMainSceneWidth(const short width);
	short getMainSceneWidth() const;
	
	// set and get functions of scene height
	void setMainSceneHeight(const short height);
	short getMainSceneHeight() const;

private:
	char *sceneName;
	short sceneWidth;
	short sceneHeight;

	// function that initialize attributes to create screen
	void init(const char *, const short, const short);
};

#endif