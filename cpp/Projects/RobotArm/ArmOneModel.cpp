#include "ArmOneModel.h"

// default constructor
ArmOneModel::ArmOneModel()
{
	// initialize textureName
	textureName = NULL;
	// call init function
	this->init();
}

// destructor
ArmOneModel::~ArmOneModel()
{
	// if textureName is not NULL, delete it
	if(NULL != textureName) {
		delete textureName;
		textureName = NULL;
	}
}

// set texture file
void ArmOneModel::setTextureFile(const char *txt)
{
	// delete textureName
	delete textureName;
	textureName = NULL;
	// allocate textureName and assign texture file name to it
	textureName = new char[MAX_CHAR];
	strcpy(textureName, txt);
}

// function that get base node
osg::Node* ArmOneModel::getArmOneNode() const 
{
	return osgDB::readNodeFile(textureName);
}

// function that initialize Node
void ArmOneModel::init()
{
	// allocate textureName and assign texture file name to it
	textureName = new char[MAX_CHAR];
	strcpy(textureName, ARMONE_TXT_PATH);
}