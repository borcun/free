/*
 * Description : Arm 3 Model Class
 * Author	   : Burak Orcun OZKABLAN
 * Date		   : 15.06.2012
 */

#ifndef ARMTHREEMODEL_H
#define ARMTHREEMODEL_H

#include <string.h>
#include <osg/ref_ptr>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include "Utility.h"

#define MAX_CHAR 256

// Arm Two Model
class ArmThreeModel : public osg::Node
{
public:
	ArmThreeModel();
	~ArmThreeModel();
	void setTextureFile(const char *);
	osg::Node* getArmThreeNode() const;

private:
	char *textureName;

	// init function
	void init();

};

#endif