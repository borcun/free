/*
 * Description : BaseMode Class
 * Author	   : Burak Orcun OZKABLAN
 * Date		   : 14.06.2012
 */

#ifndef BASEMODEL_H
#define BASEMODEL_H

#include <string.h>
#include <osg/ref_ptr>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include "Utility.h"

#define MAX_CHAR 256

// Base Model
class BaseModel : public osg::Node
{
public:
	BaseModel();
	~BaseModel();
	void setTextureFile(const char *);
	osg::Node* getBaseNode() const;

private:
	char *textureName;

	// init function
	void init();

};

#endif