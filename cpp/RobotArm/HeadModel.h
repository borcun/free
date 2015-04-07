/*
 * Description : Head Model Class
 * Author	   : Burak Orcun OZKABLAN
 * Date		   : 15.06.2012
 */

#ifndef HEADMODEL_H
#define HEADMODEL_H

#include <string.h>
#include <osg/ref_ptr>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include "Utility.h"

#define MAX_CHAR 256

// Head Model
class HeadModel : public osg::Node
{
public:
	HeadModel();
	~HeadModel();
	void setTextureFile(const char *);
	osg::Node* getHeadNode() const;

private:
	char *textureName;

	// init function
	void init();

};

#endif