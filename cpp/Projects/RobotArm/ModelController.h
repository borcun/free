/*
 * Description : Model Controller Class that handles events
 * Author	   : Burak Orcun OZKABLAN
 * Date		   : 15.06.2012
 */

#ifndef MODELCONTROLLER_H
#define MODELCONTROLLER_H

#include <vector>
#include <iostream>
#include <osg/MatrixTransform>
#include <osgGA/GUIEventHandler>
#include "Transformation.h"
#include "Model.h"
#include "Utility.h"

using namespace std;
using namespace osg;
using namespace osgGA;

// ModelController Class
class ModelController : public osgGA::GUIEventHandler
{
public:
	// constructor
	ModelController(osg::ref_ptr<osg::MatrixTransform> _baseModel,
									 osg::ref_ptr<osg::MatrixTransform> _armOneModel,
									 osg::ref_ptr<osg::MatrixTransform> _armTwoModel,
									 osg::ref_ptr<osg::MatrixTransform> _armThreeModel,
									 osg::ref_ptr<osg::MatrixTransform> _headModel);
	// destructor
	~ModelController();
	// handle function
	virtual bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);

protected:
	osg::ref_ptr<osg::MatrixTransform> baseModel;
	osg::ref_ptr<osg::MatrixTransform> armOneModel;
	osg::ref_ptr<osg::MatrixTransform> armTwoModel;
	osg::ref_ptr<osg::MatrixTransform> armThreeModel;
	osg::ref_ptr<osg::MatrixTransform> headModel;
};

#endif