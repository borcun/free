/*
 * Description  : Class that shows 3D world coordinate points.
 * Date			: 14.09.2012
 * Author		: Burak Orcun OZKABLAN
 */

#ifndef ABVISUALMODEL_H
#define ABVISUALMODEL_H

#include "ABRobotScannerUtil.h"
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>

// Visual Model Class
class ABVisualModel
{
public:
	// default constructor
    ABVisualModel();
	// constructor
    ABVisualModel(const CvMat *world_points);
    // destructor
	virtual ~ABVisualModel();
    // function that sets 3D OCV points matrix
    void setPoints(const CvMat *);
    // function that shows 3D Model
    void showModel();

private:
    osg::ref_ptr<osgViewer::CompositeViewer> m_composite_viewer;
    osg::ref_ptr<osgViewer::View> m_view;
    // allocate memory for osg view and composite viewer
    osg::ref_ptr<osg::Geometry>	m_geometry;
    osg::ref_ptr<osg::Geode> m_geode;
    // declare color array
    osg::ref_ptr<osg::Vec4Array> m_vec4Array;
    // create 3D point cloud
    osg::ref_ptr<osg::Vec3Array> m_vec3Array;
	// world coordinate points
    CvMat *m_world_points;
    // cameras position array
	osg::ref_ptr<osg::Vec3Array> m_camera_position;
	osg::ref_ptr<osg::ShapeDrawable> m_first_cam_sphere;
	osg::ref_ptr<osg::ShapeDrawable> m_second_cam_sphere; 

}; // end of class

#endif
