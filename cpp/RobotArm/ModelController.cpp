#include "ModelController.h"

// constructor
ModelController::ModelController(osg::ref_ptr<osg::MatrixTransform> _baseModel,
								 osg::ref_ptr<osg::MatrixTransform> _armOneModel,
								 osg::ref_ptr<osg::MatrixTransform> _armTwoModel,
								 osg::ref_ptr<osg::MatrixTransform> _armThreeModel,
								 osg::ref_ptr<osg::MatrixTransform> _headModel)
{
	baseModel     = _baseModel;
	armOneModel   = _armOneModel;
	armTwoModel   = _armTwoModel;
	armThreeModel = _armThreeModel;
	headModel	  = _headModel;
}

ModelController::~ModelController()
{
}

// function that handles events
bool ModelController::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	// if node is NULL, return false
	if(!baseModel || !armOneModel || !armTwoModel || !armThreeModel || !headModel)
		return false;

	cout << "X : " << ea.getX() << " , Y : " << ea.getY() << endl;

	// enter switch with event type
	switch(ea.getEventType()) {		
		// if case is osgGA::GUIEventAdapter, enter this case
		case osgGA::GUIEventAdapter::KEYDOWN:
			// enter switch with ea.getKey() function
			switch(ea.getKey()) {
				// KEY_UP
				case osgGA::GUIEventAdapter::KEY_Up:
					Transformation::transformModel(baseModel,     MOVE_TRANS, osg::Vec3(0.0f, 2.0f, 0.0f), 0);
					Transformation::transformModel(armOneModel,   MOVE_TRANS, osg::Vec3(0.0f, 2.0f, 0.0f), 0);
					Transformation::transformModel(armTwoModel,	  MOVE_TRANS, osg::Vec3(0.0f, 2.0f, 0.0f), 0);
					Transformation::transformModel(armThreeModel, MOVE_TRANS, osg::Vec3(0.0f, 2.0f, 0.0f), 0);
					Transformation::transformModel(headModel,	  MOVE_TRANS, osg::Vec3(0.0f, 2.0f, 0.0f), 0);
				break;
				// KEY_DOWN
				case osgGA::GUIEventAdapter::KEY_Down:
					Transformation::transformModel(baseModel,     MOVE_TRANS, osg::Vec3(0.0f, -2.0f, 0.0f), 0);
					Transformation::transformModel(armOneModel,   MOVE_TRANS, osg::Vec3(0.0f, -2.0f, 0.0f), 0);
					Transformation::transformModel(armTwoModel,	  MOVE_TRANS, osg::Vec3(0.0f, -2.0f, 0.0f), 0);
					Transformation::transformModel(armThreeModel, MOVE_TRANS, osg::Vec3(0.0f, -2.0f, 0.0f), 0);
					Transformation::transformModel(headModel,	  MOVE_TRANS, osg::Vec3(0.0f, -2.0f, 0.0f), 0);
				break;
				//KEY_LEFT
				case osgGA::GUIEventAdapter::KEY_Left:
					Transformation::transformModel(baseModel,	  MOVE_TRANS, osg::Vec3(-2.0f, 0.0f, 0.0f), 0);
					Transformation::transformModel(armOneModel,   MOVE_TRANS, osg::Vec3(-2.0f, 0.0f, 0.0f), 0);
					Transformation::transformModel(armTwoModel,	  MOVE_TRANS, osg::Vec3(-2.0f, 0.0f, 0.0f), 0);
					Transformation::transformModel(armThreeModel, MOVE_TRANS, osg::Vec3(-2.0f, 0.0f, 0.0f), 0);
					Transformation::transformModel(headModel,	  MOVE_TRANS, osg::Vec3(-2.0f, 0.0f, 0.0f), 0);
				break;
				// KEY_RIGHT
				case osgGA::GUIEventAdapter::KEY_Right:
					Transformation::transformModel(baseModel,	  MOVE_TRANS, osg::Vec3(2.0f, 0.0f, 0.0f), 0);
					Transformation::transformModel(armOneModel,   MOVE_TRANS, osg::Vec3(2.0f, 0.0f, 0.0f), 0);
					Transformation::transformModel(armTwoModel,	  MOVE_TRANS, osg::Vec3(2.0f, 0.0f, 0.0f), 0);
					Transformation::transformModel(armThreeModel, MOVE_TRANS, osg::Vec3(2.0f, 0.0f, 0.0f), 0);
					Transformation::transformModel(headModel,	  MOVE_TRANS, osg::Vec3(2.0f, 0.0f, 0.0f), 0);
				break;
				// ROTATE HEAD
				case 'r': case 'R':
					Transformation::transformModel(headModel, MOVE_ROT, osg::Vec3(0.0f, 10.0f, 0.0f), 0.2f);
				break;
				default:
				break;
			}
		break;
		// if double clicked
		case osgGA::GUIEventAdapter::DOUBLECLICK:
			/*cout << "X : " << ea.getX() << " , Y : " << ea.getY() << endl;*/
			Transformation::transformModel(headModel, MOVE_TRANS, osg::Vec3(ea.getX() - 500.0f , ea.getY() - 375.0f, HEAD_Z));
		break;
		// if event type is not GUI Event, break switch
		default:
		break;
	}

	// return false
	return false;
}