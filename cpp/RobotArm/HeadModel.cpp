#include "HeadModel.h"

// default constructor
HeadModel::HeadModel()
{
	// initialize textureName
	textureName = NULL;
	// call init function
	this->init();
}

// destructor
HeadModel::~HeadModel()
{
	// if textureName is not NULL, delete it
	if(NULL != textureName) {
		delete textureName;
		textureName = NULL;
	}
}

// set texture file
void HeadModel::setTextureFile(const char *txt)
{
	// delete textureName
	delete textureName;
	textureName = NULL;
	// allocate textureName and assign texture file name to it
	textureName = new char[MAX_CHAR];
	strcpy(textureName, txt);
}

// function that get base node
osg::Node* HeadModel::getHeadNode() const 
{
	return osgDB::readNodeFile(textureName);
}

// function that initialize Node
void HeadModel::init()
{
	// allocate textureName and assign texture file name to it
	textureName = new char[MAX_CHAR];
	strcpy(textureName, HEAD_TXT_PATH);
}