#include <osg/PositionAttitudeTransform>
#include <osg/StateAttribute>
#include <osg/Texture3D>
#include <osg/Image>
#include "MainScene.h"
#include "RobotArm.h"
#include "ControllerScene.h"
#include "BaseModel.h"
#include "ArmOneModel.h"
#include "ArmTwoModel.h"
#include "ArmThreeModel.h"
#include "HeadModel.h"
#include "Transformation.h"
#include "ModelController.h"
#include "Model.h"
#include "Utility.h"
#include <vector>

using namespace osg;

// getLightSource function prototype
osg::Node *getLightSource(short num, const osg::Vec3& trans, const osg::Vec4& color);

int main()
{
	cout << "Creating Scene ";

	// constant screen dimensions
	const short msWidht  = 1000;
	const short msHeight = 750;
	const short csWidht  = 250;
	const short csHeight = 750;

	// Robot Arm object
	osg::ref_ptr<RobotArm> robotArm = new RobotArm();
	// Main Scene
	osg::ref_ptr<MainScene> mainScene = new MainScene("Main Scene", msWidht, msHeight);
	// Controller Panel
	osg::ref_ptr<ControllerScene> controllerScene = new ControllerScene("Controller Scene", csWidht, csHeight);
	// create a Group object to grouping nodes of robot models
	osg::ref_ptr<osg::Group> group = new osg::Group();
	
	/******************************** NODES ********************************/
	
	// Node Models of Robot
	osg::ref_ptr<BaseModel> baseModel		  = new BaseModel();
	osg::ref_ptr<ArmOneModel> armOneModel	  = new ArmOneModel();
	osg::ref_ptr<ArmTwoModel> armTwoModel	  = new ArmTwoModel();
	osg::ref_ptr<ArmThreeModel> armThreeModel = new ArmThreeModel();
	osg::ref_ptr<HeadModel> headModel		  = new HeadModel();

	/******************************** LIGHTS **********************************/
	
	// set background colors of mainScene and controllerScene
	mainScene->getCamera()->setClearColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	controllerScene->getCamera()->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));

	// set mode of group->getOrCreateStateSet
	group->getOrCreateStateSet()->setMode( GL_LIGHT0, osg::StateAttribute::ON);
	group->getOrCreateStateSet()->setMode( GL_LIGHT1, osg::StateAttribute::ON);
	group->getOrCreateStateSet()->setMode( GL_LIGHT2, osg::StateAttribute::ON);

	// add light source to group
	group->addChild(getLightSource(0, osg::Vec3(-20.0f,0.0f,0.0f),  osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f)));;
	group->addChild(getLightSource(1, osg::Vec3(20.0f, 0.0f, 20.0f),osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)));
	group->addChild(getLightSource(2, osg::Vec3(0.0f,-30.0f,0.0f),  osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)));

	/************************ TRANSFORMATION MATRIS **************************/

	// allocate transformation vectors and add them to transVector
	osg::ref_ptr<osg::MatrixTransform> baseTrans     = new osg::MatrixTransform();
	osg::ref_ptr<osg::MatrixTransform> armOneTrans   = new osg::MatrixTransform();
	osg::ref_ptr<osg::MatrixTransform> armTwoTrans   = new osg::MatrixTransform();
	osg::ref_ptr<osg::MatrixTransform> armThreeTrans = new osg::MatrixTransform();
	osg::ref_ptr<osg::MatrixTransform> headTrans	 = new osg::MatrixTransform();

	// add nodes to transformation matrixs
	baseTrans->addChild(baseModel->getBaseNode());
	armOneTrans->addChild(armOneModel->getArmOneNode());
	armTwoTrans->addChild(armTwoModel->getArmTwoNode());
	armThreeTrans->addChild(armThreeModel->getArmThreeNode());
	headTrans->addChild(headModel->getHeadNode());


	/************************** EVENT HANDLERS *****************************/

	// create EventHandler objects
	ModelController *modelController = new ModelController(baseTrans,armOneTrans, armTwoTrans, armThreeTrans, headTrans);

	//// add event handlers to main scene
	mainScene->addEventHandler(modelController);
	//mainScene->addEventHandler(armOneModelController);
	//mainScene->addEventHandler(armTwoModelController);
	//mainScene->addEventHandler(armThreeModelController);
	//mainScene->addEventHandler(headModelController);

	// add nodes to group for initializing
	group->addChild(Transformation::transformModel( baseTrans,
													MOVE_TRANS, 
													osg::Vec3(BASE_X, BASE_Y, BASE_Z)).get()  
	);
	group->addChild(Transformation::transformModel( armOneTrans,
													MOVE_TRANS, 
													osg::Vec3(ARMONE_X, ARMONE_Y, ARMONE_Z)).get()
	);
	group->addChild(Transformation::transformModel( armTwoTrans,
													MOVE_TRANS, 
													osg::Vec3(ARMTWO_X, ARMTWO_Y, ARMTWO_Z)).get()
	);
	group->addChild(Transformation::transformModel( armThreeTrans,
													MOVE_TRANS, 
													osg::Vec3(ARMTHREE_X, ARMTHREE_Y, ARMTHREE_Z)).get()
	);
	group->addChild(Transformation::transformModel( headTrans,
													MOVE_TRANS, 
													osg::Vec3(HEAD_X, HEAD_Y, HEAD_Z)).get()
	);

	// set scene data of mainScene
	mainScene->setSceneData(group.get());

	// add mainScene to robotArm
	robotArm->addView(mainScene);
	robotArm->addView(controllerScene);

	cout << endl << "Showing Scene ";

	// realize and run robotArm
	robotArm->realize();
	robotArm->run();

	cout << endl << "Closed Scene " << endl;

	return 0;
}

// function that gets a light source with parameter
osg::Node *getLightSource(short num, const osg::Vec3& trans, const osg::Vec4& color)
{
	// create light
	osg::ref_ptr<osg::Light> light = new osg::Light();

	// set light attributes
	light->setLightNum( num );
	light->setDiffuse( color );
	light->setPosition( osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f) );

	// create a LightSource
	osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource();

	// set light of lightSource
	lightSource->setLight(light);

	// create a transform object to set light direction
	osg::ref_ptr<osg::MatrixTransform> sourceTrans = new osg::MatrixTransform();

	// set matrix sourceTrans and add child to sourceTrans
	sourceTrans->setMatrix( osg::Matrix::translate(trans) );
	sourceTrans->addChild( lightSource.get() );
	
	// return sourceTrans.release()
	return sourceTrans.release();
}