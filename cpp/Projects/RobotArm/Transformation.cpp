#include "Transformation.h"

// constructor
Transformation::Transformation()
{
}

// destructor
Transformation::~Transformation()
{
}

// function that transforms node
osg::ref_ptr<osg::MatrixTransform> Transformation::transformModel(
	osg::ref_ptr<osg::MatrixTransform> matTrans,
	const short transformation, 
	const osg::Vec3 amongVector,
	const double angle)
{
	osg::Matrix matrix = matTrans->getMatrix();

	switch(transformation) {
		case MOVE_ROT:
			// add node.get() to transform object and rotate node by amongVector and angle
			matrix *= osg::Matrix::rotate(angle, amongVector);
			matTrans->setMatrix(matrix);
			// return transform
			return matTrans;
		case MOVE_TRANS:
			// add node.get() to transform object and translate node by amongVector
			matrix *= osg::Matrix::translate(amongVector);
			matTrans->setMatrix(matrix);
			
			// return matTrans
			return matTrans;
		// if transformation wont be in any situation, return NULL
		default:
			return NULL;
	}

	return NULL;
}