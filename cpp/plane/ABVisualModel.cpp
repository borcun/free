#include "ABVisualModel.h"

using namespace osg;
using namespace osgDB;
using namespace osgGA;
using namespace osgViewer;

// default constructor
ABVisualModel::ABVisualModel()
{
    m_view = new osgViewer::View();
    m_composite_viewer = new osgViewer::CompositeViewer();
    m_geometry = new osg::Geometry();
    m_geode = new osg::Geode();
    // allocate memory for positions of cameras
	m_camera_position   = new osg::Vec3Array();
	m_first_cam_sphere  = new osg::ShapeDrawable();
	m_second_cam_sphere = new osg::ShapeDrawable();
    m_vec3Array = new osg::Vec3Array();
    m_vec4Array = new osg::Vec4Array();
}

// constructor
ABVisualModel::ABVisualModel(const CvMat *world_points)
{
	// call setPoints function to set 3D points
    setPoints(world_points);
    m_view = new osgViewer::View();
    m_composite_viewer = new osgViewer::CompositeViewer();
	// allocate memory for positions of cameras 
	m_camera_position   = new osg::Vec3Array();
	m_first_cam_sphere  = new osg::ShapeDrawable();
    m_second_cam_sphere = new osg::ShapeDrawable();
    m_vec3Array = new osg::Vec3Array();
    m_vec4Array = new osg::Vec4Array();
}

// destructor
ABVisualModel::~ABVisualModel()
{
    // release point cloud structure
    cvReleaseMat(&m_world_points);
    m_camera_position->clear();
}

// function that sets 3D points matrix
void ABVisualModel::setPoints(const CvMat *world_points)
{
	// allocate memory for points matrix
    m_world_points = cvCreateMat(world_points->rows, world_points->cols, world_points->type);
	// copy _points to points
    cvCopy(world_points, m_world_points);
}

// function that shows 3D Model
void ABVisualModel::showModel()
{
    // size of matrix
    const int size = m_world_points->rows;
    CvMat *image_point = cvCreateMat(1, 3, CV_64FC1);
    CvMat *object_point = cvCreateMat(1, 4, CV_64FC1);
    CvMat *projection_matrix = (CvMat *)cvLoad("projection_matrix.xml");

    // fill geometry object with Vec3 points
    for(int i=768 ; i < 816 ; ++i) {
        CV_MAT_ELEM(*image_point, double, 0, 0) = CV_MAT_ELEM(*m_world_points, double, i, 0);
        CV_MAT_ELEM(*image_point, double, 0, 1) = CV_MAT_ELEM(*m_world_points, double, i, 1);
        CV_MAT_ELEM(*image_point, double, 0, 2) = 1.0;
        // calculate object point
        cvMatMul(image_point, projection_matrix, object_point);

        m_vec3Array->push_back(osg::Vec3( CV_MAT_ELEM(*object_point, double, 0, 0)/* / CV_MAT_ELEM(*object_point, double, 0, 3)*/,
                                          CV_MAT_ELEM(*object_point, double, 0, 2)/* / CV_MAT_ELEM(*object_point, double, 0, 3)*/,
                                          CV_MAT_ELEM(*object_point, double, 0, 1)/* / CV_MAT_ELEM(*object_point, double, 0, 3) */)
        );

//        cout << CV_MAT_ELEM(*object_point, double, 0, 3) << " , " <<
//                CV_MAT_ELEM(*object_point, double, 0, 2) << " , " <<
//                CV_MAT_ELEM(*object_point, double, 0, 1) << endl;

        if(i / 48 % 7 == 0)
            m_vec4Array->push_back(osg::Vec4(255.0f, 255.0f, 255.0f, 0.0f));
        else if(i / 48 % 7 == 1)
            m_vec4Array->push_back(osg::Vec4(255.0f, 0.0f, 255.0f, 0.0f));
        else if(i / 48 % 7 == 2)
            m_vec4Array->push_back(osg::Vec4(255.0f, 255.0f, 0.0f, 0.0f));
        else if(i / 48 % 7 == 3)
            m_vec4Array->push_back(osg::Vec4(0.0f, 255.0f, 255.0f, 0.0f));
        else if(i / 48 % 7 == 4)
            m_vec4Array->push_back(osg::Vec4(255.0f, 0.0f, 0.0f, 0.0f));
        else if(i / 48 % 7 == 5)
            m_vec4Array->push_back(osg::Vec4(0.0f, 0.0f, 255.0f, 0.0f));
        else
            m_vec4Array->push_back(osg::Vec4(0.0f, 255.0f, 0.0f, 0.0f));
    }

    // release matrices
    cvReleaseMat(&image_point);
    cvReleaseMat(&object_point);
    cvReleaseMat(&projection_matrix);

    // set view attributes
    m_view->setUpViewInWindow(600, 300, 400, 400);
    // set vertex array and color of geometry object
    m_geometry->setVertexArray(m_vec3Array);
    m_geometry->setColorArray(m_vec4Array);
    m_geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    // add points to geometry
    m_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, m_vec3Array->size()));
    // add points to geode
    m_geode->addDrawable(m_geometry.get());
    m_view->setSceneData(m_geode.get());
    // add view to compositeViewer
    m_composite_viewer->addView(m_view);
    // run compositeViewer
    m_composite_viewer->realize();
    m_composite_viewer->run();

    return;
}
