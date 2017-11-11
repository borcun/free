#include "ArmTwoModel.h"

// default constructor
ArmTwoModel::ArmTwoModel()
{
	// initialize textureName
	textureName = NULL;
	// call init function
	this->init();
}

// destructor
ArmTwoModel::~ArmTwoModel()
{
	// if textureName is not NULL, delete it
	if(NULL != textureName) {
		delete textureName;
		textureName = NULL;
	}
}

// set texture file
void ArmTwoModel::setTextureFile(const char *txt)
{
	// delete textureName
	delete textureName;
	textureName = NULL;
	// allocate textureName and assign texture file name to it
	textureName = new char[MAX_CHAR];
	strcpy(textureName, txt);
}

// function that get base node
osg::Node* ArmTwoModel::getArmTwoNode() const 
{
	return osgDB::readNodeFile(textureName);
}

// function that initialize Node
void ArmTwoModel::init()
{
	// allocate textureName and assign texture file name to it
	textureName = new char[MAX_CHAR];
	strcpy(textureName, ARMTWO_TXT_PATH);
}