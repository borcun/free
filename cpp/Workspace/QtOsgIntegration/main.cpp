/*
 * main.cpp
 *
 *  Created on: Dec 27, 2012
 *      Author: dev01
 */

#include <QtGui/qwidget.h>
#include <QtCore/QTimer>
#include <QtGui/qapplication.h>
#include <QtGui/qgridlayout.h>
#include <Qt/qwidget.h>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#include <osgQt/GraphicsWindowQt>
#include <iostream>

using namespace osg;
using namespace osgDB;
using namespace osgGA;
using namespace osgViewer;
using namespace osgQt;
using namespace std;

// Viewer Widget Class
class ViewerWidget : public QWidget, public CompositeViewer
{
public:
	// constructor
	// the constructor have to initialize thread model of CompositeViewer constructor and calls QWidget constructor
	ViewerWidget(CompositeViewer::ThreadingModel threadingModel = CompositeViewer::SingleThreaded) : QWidget() {
		// set thread model of CompositeViewer class
        setThreadingModel(threadingModel);
        // create 4 QWidget objects whose twice have OSG screen
        QWidget *widget1 = addViewWidget(createCamera(0,0,100,100), readNodeFile("platform.osg"));
        QWidget *widget2 = addViewWidget(createCamera(0,0,100,100), readNodeFile("platform.osg"));
        QWidget *widget3 = new QWidget();
        QWidget *widget4 = new QWidget();
        // create a GridLayout to separate widget on Qt screen
        QGridLayout *grid = new QGridLayout;
        grid->addWidget(widget1, 0, 0);
        grid->addWidget(widget2, 0, 1);
        grid->addWidget(widget3, 1, 0);
        grid->addWidget(widget4, 1, 1);
        // set layout of QWidget class as grid
        setLayout(grid);
        // connect timeout of timer to update function to update QWidget
        connect(&_timer, SIGNAL(timeout()), this, SLOT(update()));
        _timer.start(1);
    }

	// function that adds view widget to CompositeViewer class
    QWidget *addViewWidget(Camera *camera, Node *node) {
        // create a View object
    	osgViewer::View* view = new osgViewer::View;
    	// set its properties
        view->setCamera(camera);
        view->setSceneData(node);
        view->addEventHandler(new StatsHandler);
        view->setCameraManipulator(new TrackballManipulator);
        // add view object to CompositeViewer
        addView(view);

        // cast graphics context of camera to GraphicsWindowsQt object
        GraphicsWindowQt *gw = dynamic_cast<GraphicsWindowQt *>(camera->getGraphicsContext());

        // if casting is successfully, return gw else return NULL
        return gw ? gw->getGLWidget() : NULL;
    }

    // function that creates a camera and return it
    Camera *createCamera(int x, int y, int w, int h, const string& name = "", bool windowDecoration = false) {
        // create a Camera reference pointer
    	ref_ptr<Camera> camera = new Camera;
    	// create a DisplaySetting object
    	DisplaySettings* ds = DisplaySettings::instance().get();
    	// create a Traits reference pointer
        ref_ptr<GraphicsContext::Traits> traits = new GraphicsContext::Traits;
        // set trait properties
        traits->windowName = name;
        traits->windowDecoration = windowDecoration;
        traits->x = x;
        traits->y = y;
        traits->width = w;
        traits->height = h;
        traits->doubleBuffer = true;
        traits->alpha = ds->getMinimumNumAlphaBits();
        traits->stencil = ds->getMinimumNumStencilBits();
        traits->sampleBuffers = ds->getMultiSamples();
        traits->samples = ds->getNumMultiSamples();
        // set graphics context of camera with traits
        camera->setGraphicsContext(new GraphicsWindowQt(traits.get()));
        // set camera properties
        camera->setClearColor( osg::Vec4(0.2, 0.2, 0.6, 1.0) );
        camera->setViewport( new osg::Viewport(0, 0, traits->width, traits->height) );
        camera->setProjectionMatrixAsPerspective(30.0f, static_cast<double>(traits->width)/static_cast<double>(traits->height), 1.0f, 10000.0f );

        // return camera object and release one in function
        return camera.release();
    }

    // paintEvent function that calls in each render process to render OSG screens
    virtual void paintEvent(QPaintEvent* event){
        // Render a complete new frame.
        // Calls advance(), eventTraversal(), updateTraversal(), renderingTraversals().
    	frame();
    }

protected:
    // QTimer object is created to render according to time
    QTimer _timer;
}; // end of class

// main function
int main( int argc, char** argv )
{
	// create a thread model to run OSG screen parallel
	// CullDrawThreadPerContext thread model run self thread for each OSG screen and cull threads
	ViewerBase::ThreadingModel threadingModel = ViewerBase::CullDrawThreadPerContext;
	// create QApplication
    QApplication app(argc, argv);
    // create a ViewerWidget object and set its thread parameter as CullDrawThreadPerContext
    ViewerWidget* viewWidget = new ViewerWidget(threadingModel);

    // set window geometry
    viewWidget->setGeometry( 100, 100, 1024, 720);
    // show window
    viewWidget->show();

    return app.exec();
} // end of main


