#include "ControllerScene.h"

// default constructors
ControllerScene::ControllerScene()
: osgViewer::View()
{
	// set sceneName by NULL
	sceneName = NULL;
}

// constructor
ControllerScene::ControllerScene(const char *name, const short width, const short height)
{
	// call init function
	this->init(name, width, height);
	
}
// destructor
ControllerScene::~ControllerScene()
{
	if(NULL != sceneName) {
		delete sceneName;
		sceneName = NULL;
	}
}

// function that sets main scene name
void ControllerScene::setControllerSceneName(const char *name)
{
	sceneName = new char[MAX_CHAR];
	strcpy(sceneName, name);
}

// function that gets main scene name
char* ControllerScene::getControllerSceneName() const
{
	return sceneName;
}

// function that sets main scene width
void ControllerScene::setControllerSceneWidth(const short width)
{
	sceneWidth = width;
}

// function that gets main scene width
short ControllerScene::getControllerSceneWidth() const
{
	return sceneWidth;
}

// function that sets main scene height
void ControllerScene::setControllerSceneHeight(const short height)
{
	sceneHeight = height;
}

// function that gets main scene height
short ControllerScene::getControllerSceneHeight() const
{
	return sceneHeight;
}

// // function that initialize View object
void ControllerScene::init(const char *name, const short width, const short height)
{
	// set parameters
	this->setControllerSceneName(name);
	this->setControllerSceneWidth(width);
	this->setControllerSceneHeight(height);

	// set view window attributes
	this->setUpViewInWindow(CS_X_POS, CS_Y_POS, width, height);

	// allocate geometry shapes
	geode		 = new osg::Geode();
	roGeomerty	 = new osg::Geometry();
	roSphere     = new osg::ShapeDrawable();
	roColor		 = new osg::Vec4Array();
	roLine	     = new osg::Vec3Array();
	tetaGeomerty = new osg::Geometry();
	tetaSphere   = new osg::ShapeDrawable();
	tetaColor	 = new osg::Vec4Array();
	tetaLine     = new osg::Vec3Array();
	fiGeomerty   = new osg::Geometry();
	fiSphere     = new osg::ShapeDrawable();
	fiColor		 = new osg::Vec4Array();
	fiLine	     = new osg::Vec3Array();

	// set roLine start and end points, color
	roLine->push_back(osg::Vec3(10.0f, 0.0f, 220.0f));
	roLine->push_back(osg::Vec3(200.0f, 0.0f, 220.0f));
	roColor->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	roGeomerty->setVertexArray(roLine.get());
	roGeomerty->setColorArray(roColor.get());
	roGeomerty->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
	roGeomerty->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2, 1));
	// set roSphere attributes and color
	roSphere = new osg::ShapeDrawable();
	roSphere->setShape( new osg::Sphere(osg::Vec3(105.0f, 0.0f, 220.0f), 5.0f) );
	roSphere->setColor( osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f) );

	// set tetaLine start and end points, color 
	tetaLine->push_back(osg::Vec3(10.0f, 0.0f, 180.0f));
	tetaLine->push_back(osg::Vec3(200.0f, 0.0f, 180.0f));
	tetaColor->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	tetaGeomerty->setVertexArray(tetaLine.get());
	tetaGeomerty->setColorArray(tetaColor.get());
	tetaGeomerty->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
	tetaGeomerty->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2, 1));
	// set tetaSphere attributes and color
	tetaSphere = new osg::ShapeDrawable();
	tetaSphere->setShape( new osg::Sphere(osg::Vec3(105.0f, 0.0f, 180.0f), 5.0f) );
	tetaSphere->setColor( osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );

	// set fiSphere start and end points, color
	fiLine->push_back(osg::Vec3(10.0f, 0.0f, 140.0f));
	fiLine->push_back(osg::Vec3(200.0f, 0.0f, 140.0f));
	fiColor->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	fiGeomerty->setVertexArray(fiLine.get());
	fiGeomerty->setColorArray(fiColor.get());
	fiGeomerty->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
	fiGeomerty->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2, 1));
	// set fiSpehere attributes and color
	fiSphere = new osg::ShapeDrawable();
	fiSphere->setShape( new osg::Sphere(osg::Vec3(105.0f, 0.0f, 140.0f), 5.0f) );
	fiSphere->setColor( osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) );

	// create texts
	osg::ref_ptr<osgText::Text> roText	 = new osgText::Text();
	osg::ref_ptr<osgText::Text> tetaText = new osgText::Text();
	osg::ref_ptr<osgText::Text> fiText	 = new osgText::Text();
	// set text color, position and its text of roText
	roText->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
	roText->setCharacterSize(15.0f);
	roText->setAxisAlignment( osgText::TextBase::XZ_PLANE );
	roText->setPosition(osg::Vec3(10.0f, 0.0f, 235.0f));
	roText->setText("r");
	// set text color, position and its text of tetaText
	tetaText->setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	tetaText->setCharacterSize(15.0f);
	tetaText->setAxisAlignment( osgText::TextBase::XZ_PLANE );
	tetaText->setPosition(osg::Vec3(10.0f, 0.0f, 195.0f));
	tetaText->setText("teta");
	// set text color, position and its text of fiText
	fiText->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
	fiText->setCharacterSize(15.0f);
	fiText->setAxisAlignment( osgText::TextBase::XZ_PLANE );
	fiText->setPosition(osg::Vec3(10.0f, 0.0f, 155.0f));
	fiText->setText("fi");

	// add lines and texts to geode
	geode->addDrawable(roText.get());
	geode->addDrawable(roGeomerty.get());
	geode->addDrawable(tetaText.get());
	geode->addDrawable(tetaGeomerty);	
	geode->addDrawable(fiText.get());
	geode->addDrawable(fiGeomerty);
	// add spheres to geode
	geode->addDrawable(roSphere.get());
	geode->addDrawable(tetaSphere.get());
	geode->addDrawable(fiSphere.get());

	// add geode to view
	this->setSceneData(geode.get());

	return;
}