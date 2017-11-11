/*
 * Description : Arm 1 Model Class
 * Author	   : Burak Orcun OZKABLAN
 * Date		   : 15.06.2012
 */

#ifndef ARMONEMODEL_H
#define ARMONEMODEL_H

#include <string.h>
#include <osg/ref_ptr>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include "Utility.h"

#define MAX_CHAR 256

// Arm One Model
class ArmOneModel : public osg::Node
{
public:
	ArmOneModel();
	~ArmOneModel();
	void setTextureFile(const char *);
	osg::Node* getArmOneNode() const;

private:
	char *textureName;

	// init function
	void init();

};

#endif