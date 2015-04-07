/*
 * Description : Program calibrates camera with a video from camera and used chessboard.
 *				 Parameter(s) from out are square size, horizontal and vertical corners counts of chessboard
 * Author	   : Burak Orcun OZKABLAN
 * Date		   : 24.08.2012
 */

#ifndef ABCALIBRATION_H
#define ABCALIBRATION_H

#include "ABRobotScannerUtil.h"
#include <dirent.h>
#include <unistd.h>

#define SECOND_CALIBRATION 1

// camera number of calibration process
enum {LEFT_CAM = 1, RIGHT_CAM};

class ABCalibration
{
public:
	// default constructor
	ABCalibration();
	// constructor
    explicit ABCalibration(const char *dir_path);
	// destructor
	virtual ~ABCalibration();
    // function that set path of image
    void setImagePath(const char *dir_path);
	// function that set attributes of chessboard
	void setChessboardAttributes(const double chessboard_square_size, const int chessboard_row_corner_count, const int chessboard_column_corner_count);
	// function that runs calibration
	void run(const int which_camera);

private:
	CvMat *m_object_points;
    CvMat *m_image_points;
	CvMat *m_found_points;
	CvSize m_image_size;
	CvSize m_board_size;
	CvMat *m_intrinsic_matrix;
	CvMat *m_dis_coefficient;
    CvMat *m_rotation_matrix;
    CvMat *m_rotation_vector;
    CvMat *m_translation_vector;
    CvMat *m_projection_matrix;
	// attributes of chessboard
	double m_square_size;
	// horizontal(column) and vertical(row) corners counts
	int m_column_count;
	int m_row_count;
    // images path
    char m_dir_path[MAX_CHAR];

}; // end of class

#endif
