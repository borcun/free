#include "ABVisualModel.hpp"

// default constructor
ABVisualModel::ABVisualModel() : QGLWidget(), m_scale(PC_WIDTH / 10)
{
	m_x_rotation = m_y_rotation = m_z_rotation = 0.0f;
}

// constructor
ABVisualModel::ABVisualModel(const pcl::PointCloud<pcl::PointXYZ> &world_points) : QGLWidget(), m_scale(PC_WIDTH / 10)
{
	m_x_rotation = m_y_rotation = m_z_rotation = 0.0f;
    // call setPoints function to set 3D points
    setPCLPoints(world_points);
}

// destructor
ABVisualModel::~ABVisualModel()
{
    // release point cloud structure
    cvReleaseMat(&m_ocv_world_points);
    m_pcl_world_points.clear();
}

// function that sets 3D PCL points matrix
void ABVisualModel::setPCLPoints(const pcl::PointCloud<pcl::PointXYZ> &pcl_world_points)
{
    m_pcl_world_points = pcl_world_points;
}

// function that initializes GL
void ABVisualModel::initializeGL()
{
    static const GLfloat lightPos[4] = { 5.0f, 5.0f, 10.0f, 1.0f };
    static const GLfloat reflectance1[4] = { 0.8f, 0.1f, 0.0f, 1.0f };
    static const GLfloat reflectance2[4] = { 0.0f, 0.8f, 0.2f, 1.0f };
    static const GLfloat reflectance3[4] = { 0.2f, 0.2f, 1.0f, 1.0f };

    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glEnable(GL_NORMALIZE);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);  
	//glClearColor(0.85f, 0.85f, 0.85f, 0.f);
	resizeGL(PC_WIDTH, PC_HEIGHT);

    glShadeModel( GL_SMOOTH );
    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
    glEnable( GL_TEXTURE_2D );
    glEnable( GL_CULL_FACE );
    glEnable( GL_DEPTH_TEST );

	return;
}

// function that paints GL
void ABVisualModel::paintGL()
{
    makeCurrent();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(0.0, 0.0, -10.0);
    glRotated(m_x_rotation / 16.0, 1.0, 0.0, 0.0);
    glRotated(m_y_rotation / 16.0, 0.0, 1.0, 0.0);
    glRotated(m_z_rotation / 16.0, 0.0, 0.0, 1.0);
	glScalef(m_scale / PC_WIDTH, m_scale / PC_WIDTH, m_scale / PC_WIDTH);
    glShadeModel(GL_FLAT);
    glMatrixMode( GL_MODELVIEW );
    glBegin(GL_POINTS);
    glPointSize(5.0);
		// if ocv is ready
		if(0 == m_pcl_world_points.size()) {
			for(int i = 0 ; i < m_ocv_world_points->cols ; ++i) {
				glColor4f(1.0, 1.0, 1.0, 1);
				glVertex3f( CV_MAT_ELEM(*m_ocv_world_points, float, 0, i) / CV_MAT_ELEM(*m_ocv_world_points, float, 3, i), 
						    CV_MAT_ELEM(*m_ocv_world_points, float, 1, i) / CV_MAT_ELEM(*m_ocv_world_points, float, 3, i), 
							CV_MAT_ELEM(*m_ocv_world_points, float, 2, i) / CV_MAT_ELEM(*m_ocv_world_points, float, 3, i));
			}
		}
		// if pcl is ready
		else {
			for(int i = 0 ; i < (int)m_pcl_world_points.size() ; ++i) {
				glColor4f(1.0, 1.0, 1.0, 1);
				glVertex3f(m_pcl_world_points.points.at(i).x, m_pcl_world_points.points.at(i).y, m_pcl_world_points.points.at(i).z);
			}
		}
    glEnd();
    glPopMatrix();
}

// function that resize GL
void ABVisualModel::resizeGL(const int width, const int height)
{
	//proces resize keep good aspect ratio for 3D scene
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    int side = qMin(width, height);
    //glViewport((width - side) / 2, (height - side) / 2, side, side);
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-1.0, +1.0, -1.0, 1.0, 5.0, 60.0);
    //gluPerspective(45.0, (GLfloat)width/(GLfloat)height, 0.01f, 1000.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslated(0.0, 0.0, -40.0);
}

void ABVisualModel::mousePressEvent(QMouseEvent *event)
{
	//proces mouse events for rotate/move inside 3D scene
    m_last_pos = event->pos();
}

void ABVisualModel::mouseMoveEvent(QMouseEvent *event)
{
    //proces keyboard events
    int dx = event->x() - m_last_pos.x();
    int dy = event->y() - m_last_pos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(m_x_rotation + 8 * dy);
        setYRotation(m_y_rotation + 8 * dx);
    } 
	else if (event->buttons() & Qt::RightButton) {
        setXRotation(m_x_rotation + 8 * dy);
        setZRotation(m_z_rotation + 8 * dx);
    }

    m_last_pos = event->pos();
}

void ABVisualModel::wheelEvent(QWheelEvent *e)
{
     e->delta() > 0 ? m_scale += m_scale*0.1f : m_scale -= m_scale*0.1f;
     updateGL();
}

static void qNormalizeAngle(int &angle)
 {
     while (angle < 0)
         angle += 360 * 16;
     while (angle > 360 * 16)
         angle -= 360 * 16;
 }

 void ABVisualModel::setXRotation(int angle)
 {
     qNormalizeAngle(angle);
     if (angle != m_x_rotation) {
         m_x_rotation = angle;
         //emit xRotationChanged(angle);
         //updateGL();
     }
 }

 void ABVisualModel::setYRotation(int angle)
 {
     qNormalizeAngle(angle);
     if (angle != m_y_rotation) {
         m_y_rotation = angle;
         //emit yRotationChanged(angle);
         //updateGL();
     }
 }

 void ABVisualModel::setZRotation(int angle)
 {
     qNormalizeAngle(angle);
     if (angle != m_z_rotation) {
         m_z_rotation = angle;
         //emit zRotationChanged(angle);
         //updateGL();
     }
 }

// function that updates world point cloud
void ABVisualModel::updatePointCloud(pcl::PointCloud<pcl::PointXYZ> &pcl_world_points)
{
    m_pcl_world_points = pcl_world_points;
    updateGL();
}

// function that calculate positions of cameras
void ABVisualModel::showCamerasPositions(const CvMat *rotation_matrix, const CvMat *translation_matrix)
{
	CvMat *first_cam_pos  = cvCreateMat(3, 1, CV_64FC1);
	CvMat *second_cam_pos = cvCreateMat(3, 1, CV_64FC1);

	// set first camera origin point
	CV_MAT_ELEM(*first_cam_pos, double, 0, 0) = IMG_WIDTH / 2;
	CV_MAT_ELEM(*first_cam_pos, double, 1, 0) = IMG_HEIGHT / 2;
	CV_MAT_ELEM(*first_cam_pos, double, 2, 0) = 1.0;
	// calculate second camera origin point
	cvMatMul(rotation_matrix, first_cam_pos, second_cam_pos);
	cvAdd(second_cam_pos, translation_matrix, second_cam_pos);

	// add positions of cameras to camera_position vector
	// ..............

	// release matrices
	cvReleaseMat(&first_cam_pos);
	cvReleaseMat(&second_cam_pos);

	return;
}
