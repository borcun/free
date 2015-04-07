#include "MainScene.h"

// default constructors
MainScene::MainScene()
: osgViewer::View()
{
	// set sceneName by NULL
	sceneName = NULL;
}

// constructor
MainScene::MainScene(const char *name, const short width, const short height)
{
	// set scene data, scene size and coordinate
	this->init(name, width, height);
}
// destructor
MainScene::~MainScene()
{
	if(NULL != sceneName) {
		delete sceneName;
		sceneName = NULL;
	}
}

// function that sets main scene name
void MainScene::setMainSceneName(const char *name)
{
	sceneName = new char[MAX_CHAR];
	strcpy(sceneName, name);
}

// function that gets main scene name
char* MainScene::getMainSceneName() const
{
	return sceneName;
}

// function that sets main scene width
void MainScene::setMainSceneWidth(const short width)
{
	sceneWidth = width;
}

// function that gets main scene width
short MainScene::getMainSceneWidth() const
{
	return sceneWidth;
}

// function that sets main scene height
void MainScene::setMainSceneHeight(const short height)
{
	sceneHeight = height;
}

// function that gets main scene height
short MainScene::getMainSceneHeight() const
{
	return sceneHeight;
}

// function that initialize View object
void MainScene::init(const char *name, const short width, const short height)
{	
	// set parameters
	this->setMainSceneName(name);
	this->setMainSceneWidth(width);
	this->setMainSceneHeight(height);

	// set view window attributes
	this->setUpViewInWindow(MS_X_POS, MS_Y_POS, width, height);
}