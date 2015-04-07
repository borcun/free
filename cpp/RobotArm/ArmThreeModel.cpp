#include "ArmThreeModel.h"

// default constructor
ArmThreeModel::ArmThreeModel()
{
	// initialize textureName
	textureName = NULL;
	// call init function
	this->init();
}

// destructor
ArmThreeModel::~ArmThreeModel()
{
	// if textureName is not NULL, delete it
	if(NULL != textureName) {
		delete textureName;
		textureName = NULL;
	}
}

// set texture file
void ArmThreeModel::setTextureFile(const char *txt)
{
	// delete textureName
	delete textureName;
	textureName = NULL;
	// allocate textureName and assign texture file name to it
	textureName = new char[MAX_CHAR];
	strcpy(textureName, txt);
}

// function that get base node
osg::Node* ArmThreeModel::getArmThreeNode() const 
{
	return osgDB::readNodeFile(textureName);
}

// function that initialize Node
void ArmThreeModel::init()
{
	// allocate textureName and assign texture file name to it
	textureName = new char[MAX_CHAR];
	strcpy(textureName, ARMTHREE_TXT_PATH);
}