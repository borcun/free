#include "BaseModel.h"

// default constructor
BaseModel::BaseModel()
{
	// initialize textureName
	textureName = NULL;
	// call init function
	this->init();
}

// destructor
BaseModel::~BaseModel()
{
	// if textureName is not NULL, delete it
	if(NULL != textureName) {
		delete textureName;
		textureName = NULL;
	}
}

// set texture file
void BaseModel::setTextureFile(const char *txt)
{
	// delete textureName
	delete textureName;
	textureName = NULL;
	// allocate textureName and assign texture file name to it
	textureName = new char[MAX_CHAR];
	strcpy(textureName, txt);
}

// function that get base node
osg::Node* BaseModel::getBaseNode() const 
{
	return osgDB::readNodeFile(textureName);
}

// function that initialize Node
void BaseModel::init()
{
	// allocate textureName and assign texture file name to it
	textureName = new char[MAX_CHAR];
	strcpy(textureName, BASE_TXT_PATH);
}