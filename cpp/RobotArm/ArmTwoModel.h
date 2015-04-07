/*
 * Description : Arm 2 Model Class
 * Author	   : Burak Orcun OZKABLAN
 * Date		   : 15.06.2012
 */

#ifndef ARMTWOMODEL_H
#define ARMTWOMODEL_H

#include <string.h>
#include <osg/ref_ptr>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include "Utility.h"

#define MAX_CHAR 256

// Arm Two Model
class ArmTwoModel : public osg::Node
{
public:
	ArmTwoModel();
	~ArmTwoModel();
	void setTextureFile(const char *);
	osg::Node* getArmTwoNode() const;

private:
	char *textureName;

	// init function
	void init();

};

#endif