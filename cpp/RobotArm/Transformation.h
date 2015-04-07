/*
 * Description : Transformation Class
 * Author	   : Burak Orcun OZKABLAN
 * Date		   : 15.06.2012
 */

#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <vector>
#include <osg/Vec3>
#include <osg/Matrix>
#include <osg/Transform>
#include <osg/MatrixTransform>
#include "Utility.h"
#include "Model.h"

using namespace osg;

// Transformation Class
class Transformation : public osg::MatrixTransform
{
public:
	// constructor
	Transformation();
	// destructor
	~Transformation();
	// function that transform model
	static osg::ref_ptr<osg::MatrixTransform> transformModel(
		osg::ref_ptr<osg::MatrixTransform> matTrans,
		const short transformation, 
		const osg::Vec3 amongVector,
		const double angle = 0.0
	);
};

#endif