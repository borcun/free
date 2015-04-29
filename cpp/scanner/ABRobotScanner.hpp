/**
 * @file:   ABRobotScanner.hpp
 * @title:  ABRobotScanner Header
 * @author: Burak Orcun OZKABLAN
 * @date:   01.04.2013
 */

#ifndef ABROBOTSCANNER_HPP
#define ABROBOTSCANNER_HPP

#include <QtGui/QWidget>
#include "ui_ABRobotScanner.h"
#include "ABxiQ.hpp"
#include "ABCalibration.hpp"
#include "ABFeatureDetector.hpp"
#include "ABFundamentalMatrix.hpp"
#include "ABEpipolarLine.hpp"
#include "ABEssentialMatrix.hpp"
#include "ABReconstruction.hpp"
#include "ABVisualModel.hpp"

class ABRobotScanner : public QWidget
{
	Q_OBJECT

public:
	/// \brief constructor
	/// @param parent - Widget pointer
	/// @param flags - Widget flags
	ABRobotScanner(QWidget *parent = 0, Qt::WFlags flags = 0);
	/// \brief destructor
	~ABRobotScanner();

private:
	Ui::ABRobotScannerClass ui;
	// capture objects
    ABxiQ *m_left_capture;
    ABxiQ *m_right_capture;
	// calibration objects
	ABCalibration *m_ab_calibration_1;
    ABCalibration *m_ab_calibration_2;
	// feature detector object
	ABFeatureDetector *m_ab_fd;
	// fundamental matrix
	ABFundamentalMatrix *m_ab_fm;
	// epipolar lines objects
	ABEpipolarLine *m_ab_epilines_1;
	ABEpipolarLine *m_ab_epilines_2;
	// essential matrix object
	ABEssentialMatrix *m_ab_em;
	// reconstruction object
	ABReconstruction *m_ab_reconstruction;
	// visual model object
	ABVisualModel *m_ab_vm;
	// surface points
	pcl::PointCloud<pcl::PointXYZ> m_surface_points;
	// image objects
    boost::shared_ptr<IplImage> m_left_image;
    boost::shared_ptr<IplImage> m_right_image;
    IplImage *m_left_temp_image;
    IplImage *m_right_temp_image;
	// buffers
	void *buffer_1;
	void *buffer_2;
	// reconstruction thread
    boost::thread m_reconstruction_t[1];
    // reconstruction matrices
    CvMat *m_intrinsic_matrix_1;
    CvMat *m_intrinsic_matrix_2;
    CvMat *m_dis_coef_1;
    CvMat *m_dis_coef_2;
    CvMat *m_fundamental_matrix;
    CvMat *m_essential_matrix;
    CvMat *m_rotation_matrix_1;
    CvMat *m_rotation_matrix_2;
    CvMat *m_translation_matrix_1;
    CvMat *m_translation_matrix_2;
    CvMat *m_projection_matrix_1;
    CvMat *m_projection_matrix_2;
    CvMat *m_world_points;

    /// \brief function that initializes fundamental materials
	/// @return -
    void init();
    /// \brief function that connects signals to slots
	/// @return -
    void connectSignalsToSlots();
    /// \brief function that detects feature of images
    /// @return -
	void detectFeatures();
    ///\brief function that calculates fundamental matrix
    /// @return -
	void fundamentalMatrix();
    /// \brief function that estimates and draws epipolar lines
    /// @return -
	void epipolarLines();
    /// \brief function that finds essential matrix
    /// @return -
	void essentialMatrix();
    /// \brief function that finds rotation and translation matrices
    /// @return -
	void rotationTranslation();
    /// \brief function that estimates projection matrices
    /// @return -
	void projectionMatrices();
    /// \brief function that captures frame from left camera
	/// @param dir_name - directory name is image capturing directory
	/// @param file_name - file name is image capturing name
	/// @return -
    void leftCameraCapturing(const string dir_name, const string file_name = "");
    /// \brief function that captures frame from right camera
	/// @param dir_name - directory name is image capturing directory
	/// @param file_name - file name is image capturing name
	/// @return -
    void rightCameraCapturing(const string dir_name, const string file_name = "");
    /// \brief function that captures frames as independent from startSystem function
    /// @return -
	void setReconstructionDataset();
    /// \brief function that converts IplImage to QImage
	/// @param ipl_image - IplImage that will be converted to QImage
	/// @param width - image width
	/// @param height - image height
	/// @return QImage
    QImage iplImage2QImage(const IplImage *ipl_image, const int width, const int height);
    /// \brief function that converts IplImage to QImage
	/// @param ipl_image - IplImage that will be converted to QImage
	/// @param width - image width
	/// @param height - image height
	/// @return QImage
    QImage xiQImage2QImage(const XI_IMG xiQ_image, const int width, const int height);
	/// \brief function that save point could by .xyz file extension format
	/// @param file_name - xyz file name
	/// @param point_cloud - point cloud that is saved as xyz file
	/// @return boolean
	bool savePointCloudXYZFormat(const char *file_name, pcl::PointCloud<pcl::PointXYZ> &point_cloud);
	/// \brief function that print matrix on screen
	/// @param matrix_name - matrix name
	/// @param matrix - matrix
	/// @return -
    void printMatrix(const char *matrix_name, const CvMat *matrix);

public slots:
    /// \brief function that starts system
	/// @return -
    void startSystem();
    /// \brief function that starts camera
    /// @return -
	void stream();
	/// \brief function that refines system
	/// @return -
	void refineSystem();
    /// \brief function that calibrates cameras
	/// @return boolean
    bool calibration();
    /// \brief function that captures frames from two cameras
	/// @return -
    void twoCameraCapturing();
    /// \brief function that clears cache
	/// @return -
    void clearCache();
    /// \brief function that shows an article about how to space and camera calibrating
    /// @return -
	void howToCalibrating();
    /// \brief function that reconstructs 3D space of object
    /// @return -
	void reconstruction();
    /// \brief function that shows visual model of 3D object
	/// @return -
    void visualModel();
    /// \brief function that saves point cloud of object
	/// @return -
    void savePointCloud();
    /// \brief function that shows help file
	/// @return -
    void helpFile();
    /// \brief function that terminates application
	/// @return -
    void exit();
};

#endif // ABROBOTSCANNER_HPP
