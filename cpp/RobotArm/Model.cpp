#include "Model.h"

// constructor
Model::Model()
{
	name = new char[MAX_CHAR];
	matrix = new osg::MatrixTransform();
}

// constructor
Model::Model(const osg::ref_ptr<osg::MatrixTransform> _matrix, const short _number, 
				const char * _name, const short _x, const short _y, const short _z, const double _angle)
{
	name = new char[MAX_CHAR];
	matrix = new osg::MatrixTransform();

	this->setMatrix(_matrix);
	this->setNumber(_number);
	this->setName(_name);
	this->setX(_x);
	this->setY(_y);
	this->setZ(_z);
	this->setAngle(_angle);
}

Model::Model(Model *_model)
{
	name = new char[MAX_CHAR];
	matrix = new osg::MatrixTransform();

	this->setMatrix(_model->getMatrix());
	this->setNumber(_model->getNumber());
	this->setName(_model->getName());
	this->setX(_model->getX());
	this->setY(_model->getY());
	this->setZ(_model->getZ());
	this->setAngle(_model->getAngle());
}

// destructor
Model::~Model()
{
	delete name;
	name = NULL;
}

void Model::setMatrix(const osg::ref_ptr<osg::MatrixTransform> _matrix)
{
	matrix = _matrix;
}

osg::ref_ptr<osg::MatrixTransform> Model::getMatrix() const
{
	return matrix;
}

// set and get functions
void Model::setNumber(const short _number)
{
	number = _number;
}

short Model::getNumber() const
{
	return number;
}

void Model::setName(const char *_name)
{
	strcpy(name, _name);
}

char *Model::getName() const
{
	return name;
}

void Model::setX(const short _x)
{
	x = _x;
}

short Model::getX() const
{
	return x;
}

void Model::setY(const short _y)
{
	y = _y;
}

short Model::getY() const
{
	return y;
}

void Model::setZ(const short _z)
{
	z = _z;
}

short Model::getZ() const
{
	return z;
}

void Model::setAngle(const double _angle)
{
	angle = _angle;
}

double Model::getAngle() const
{
	return angle;
}
