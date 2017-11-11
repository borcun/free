#ifndef MODEL_H
#define MODEL_H

#include <osg/MatrixTransform>
#include <string.h>
#define MAX_CHAR 256

using namespace osg;

class Model 
{
public:
	// constructor
	Model();
	Model(const osg::ref_ptr<osg::MatrixTransform>, const short, const char *, const short, const short, const short, const double);
	// copy constructor
	Model(Model *);
	// destructor
	~Model();
	// set and get functions
	void setMatrix(const osg::ref_ptr<osg::MatrixTransform>);
	osg::ref_ptr<osg::MatrixTransform> getMatrix() const;
	void setNumber(const short);
	short getNumber() const;
	void setName(const char *);
	char *getName() const;
	void setX(const short);
	short getX() const;
	void setY(const short);
	short getY() const;
	void setZ(const short);
	short getZ() const;
	void setAngle(const double);
	double getAngle() const;

private:
	osg::ref_ptr<osg::MatrixTransform> matrix;
	short number;
	char *name;
	short x;
	short y;
	short z;
	double angle;
};

#endif