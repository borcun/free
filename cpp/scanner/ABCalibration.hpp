/**
 * @file:   ABCalibration.h
 * @title:  Program calibrates camera with a video from camera and used chessboard.
 *			Parameter(s) from out are square size, horizontal and vertical corners counts of chessboard
 * @author: Burak Orcun OZKABLAN
 * @date:   24.08.2012
 */

#ifndef ABCALIBRATION_H
#define ABCALIBRATION_H

#include "ABRobotScannerUtil.hpp"
#include <QDebug>
#include <QMessageBox>

#define SECOND_CALIBRATION 0
#define CALIBRATION_DATASET 22

// camera number of calibration process
enum {LEFT_CAM = 1, RIGHT_CAM};

class ABCalibration
{
public:
	/// \brief default constructor
	ABCalibration();
	/// \brief constructor
	/// @param dir_path - directory path that contains of calibration images
    explicit ABCalibration(const char *dir_path);
	/// \brief destructor
	virtual ~ABCalibration();
    /// \brief function that set path of image
	/// @param dir_path - directory path that contains of calibration images
	/// @return -
	void setImagePath(const char *dir_path);
	/// \brief function that set attributes of chessboard
	/// @param chessboard_square_size - an edge length of square by cm
	/// @param chessboard_row_corner_count - row corners count of chessboard
	/// @param chessboard_column_corner_count - column corners count of chessboard
	/// @return -
	void setChessboardAttributes(const double chessboard_square_size, const int chessboard_row_corner_count, const int chessboard_column_corner_count);
	/// \brief function that runs calibration
	/// @param which_camera - camera index
	/// @return -
	void run(const int which_camera);

private:
	CvMat *m_object_points;
    CvMat *m_image_points;
	CvMat *m_found_points;
	CvSize m_image_size;
	CvSize m_board_size;
	CvMat *m_intrinsic_matrix;
	CvMat *m_dis_coefficient;
	double m_square_size;
	int m_column_count;
	int m_row_count;
    char m_dir_path[MAX_CHAR];

};

#endif
