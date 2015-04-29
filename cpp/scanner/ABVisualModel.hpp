/**
 * @file:   ABVisualModel.hpp
 * @title:  Class that shows 3D world coordinate points.
 * @author: Burak Orcun OZKABLAN
 * @date:   14.09.2012
 */

#ifndef ABVISUALMODEL_H
#define ABVISUALMODEL_H

#include "ABRobotScannerUtil.hpp"
#include <qwidget.h>
#include <QtGui/QGridLayout>
#include <QtOpenGL/QGLWidget>
#include <QtOpenGL/QtOpenGL>

// Visual Model Class
class ABVisualModel : public QGLWidget
{
public:
	/// \brief default constructor
    ABVisualModel();
    /// \brief constructor
	/// @param world_points - world coordinate points by PCL PointCloud<T>
    ABVisualModel(const pcl::PointCloud<pcl::PointXYZ> &world_points);
    /// \brief destructor
	virtual ~ABVisualModel();
    /// \brief function that sets 3D PCL points matrix
	/// @param world_points - world coordinate points
	/// @return -
    void setPCLPoints(const pcl::PointCloud<pcl::PointXYZ> &world_points);
	/// \brief function that calculates positions of cameras
	/// @param rotation_matrix - rotation matrix
	/// @param translation_matrix - translation matrix 
	/// @return -
	void showCamerasPositions(const CvMat *rotation_matrix, const CvMat *translation_matrix);
	/// \brief function that resizes GL
	/// @return -
	void resizeGL(const int width, const int height);
    /// \brief function that updates point cloud
	/// @return -
	void updatePointCloud(pcl::PointCloud<pcl::PointXYZ> &pcl_world_points);

protected:
	/// \brief function that initializes GL
	/// @return -
	void initializeGL();
    /// \brief function that paints GL
	/// @return -
	void paintGL();
    /// \brief function that is event of pressing mouse
	/// @param event - Qt mouse event
	/// @return -	
	void mousePressEvent(QMouseEvent *event);
    /// \brief function that is event of moving mouse
	/// @param event - Qt mouse event
	/// @return -	
	void mouseMoveEvent(QMouseEvent *event);
    /// \brief function that is event of scrolling mouse
	/// @param event - Qt mouse event
	/// @return -
	void wheelEvent(QWheelEvent *e);

private:
	// world coordinate points
	CvMat *m_ocv_world_points;
	pcl::PointCloud<pcl::PointXYZ> m_pcl_world_points;
    QPoint m_last_pos;
	// rotation angles
	float m_x_rotation, m_y_rotation, m_z_rotation;
    float m_scale;

public slots:
	/// \brief function that sets x rotation
	/// @param angle - x rotation angle
	/// @return -
	void setXRotation(int angle);
	/// \brief function that sets y rotation
	/// @param angle - x rotation angle
	/// @return -
	void setYRotation(int angle);
	/// \brief function that sets z rotation
	/// @param angle - x rotation angle
	/// @return -
	void setZRotation(int angle);

}; // end of class

#endif
