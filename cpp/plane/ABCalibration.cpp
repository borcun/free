#include "ABCalibration.h"

// default constructor
ABCalibration::ABCalibration()
{
	m_dis_coefficient  = cvCreateMat(5, 1, CV_64FC1);
    m_intrinsic_matrix = cvCreateMat(3, 3, CV_64FC1);
    m_rotation_matrix = cvCreateMat(3, 3, CV_64FC1);
    m_rotation_vector = cvCreateMat(3, 1, CV_64FC1);
    m_translation_vector = cvCreateMat(3, 1, CV_64FC1);
    m_projection_matrix = cvCreateMat(3, 4, CV_64FC1);
}

// constructor
ABCalibration::ABCalibration(const char *dir_path)
{
	m_intrinsic_matrix = cvCreateMat(3, 3, CV_64FC1);
	m_dis_coefficient  = cvCreateMat(5, 1, CV_64FC1);
    m_rotation_matrix = cvCreateMat(3, 3, CV_64FC1);
    m_rotation_vector = cvCreateMat(3, 1, CV_64FC1);
    m_translation_vector = cvCreateMat(3, 1, CV_64FC1);
    m_projection_matrix = cvCreateMat(3, 4, CV_64FC1);
    // set images directory path
    setImagePath(dir_path);
}

// destructor
ABCalibration::~ABCalibration()
{
	cvReleaseMat(&m_intrinsic_matrix);
	cvReleaseMat(&m_dis_coefficient);
    cvReleaseMat(&m_rotation_matrix);
    cvReleaseMat(&m_rotation_vector);
    cvReleaseMat(&m_translation_vector);
    cvReleaseMat(&m_projection_matrix);
    cvReleaseMat(&m_object_points);
    cvReleaseMat(&m_image_points);
	cvReleaseMat(&m_found_points);
}

// function that sets video path
void ABCalibration::setImagePath(const char *dir_path)
{
    // set images directory path
    strcpy(m_dir_path, dir_path);
}

// function that sets attributes of chessboard
void ABCalibration::setChessboardAttributes(const double square_size, const int row_count, const int column_count)
{
	// set one squares size of chessboard
	m_square_size = square_size;
	// set horizontal(column) corners and vertical(row) corners counts of chessboard
	m_row_count = row_count;
	m_column_count = column_count;
}

// function that runs calibration
void ABCalibration::run(const int which_camera)
{
	// frame from capture
	IplImage *frame;
	// gray frame from capture
	IplImage *gray_frame;
	// image points vector
	vector<CvPoint2D32f> image_vector;
	// all corners points count
	const int point_count = m_row_count * m_column_count;
	// total corners in a frame
	CvPoint2D32f *corners = new CvPoint2D32f[point_count];
	// points counts that can be found in a frame
	int corner_count;
	// found corner flag
	int found_flag;
	// frame count
	int frame_count = 0;
	// re-projection error
	double error;
    // create a 2D point object
    CvPoint2D32f point;
    // direcorty path
    DIR *dir;
    struct dirent *dir_entity;
    // image file path
    char file_path[MAX_CHAR];

    // open images directory path
    if((dir = opendir(m_dir_path)) == NULL) {
        return;
    }

    // set image size
    m_image_size = cvSize(IMG_WIDTH, IMG_HEIGHT);
    // allocate memory for frame
    frame = cvCreateImage(m_image_size, IPL_DEPTH_8U, 3);
    // create gray scale image from frame
    gray_frame = cvCreateImage(m_image_size, IPL_DEPTH_8U, 1);
	// set board size by ( column x row )
	m_board_size = cvSize(m_column_count, m_row_count);
    
    cvNamedWindow("Kalibrasyon");
            
    // pick up points of chess board as sub-pixel precision
    while ((dir_entity = readdir(dir)) != NULL) {
        // prevent current and parent directories
        if(strcmp(dir_entity->d_name, ".") && strcmp(dir_entity->d_name, "..")) {
            // zero file_path
            bzero(file_path, MAX_CHAR);
            // set image file path
            strcpy(file_path, m_dir_path);
            strcat(file_path, "/");
            strcat(file_path, dir_entity->d_name);
            // load image to frame
            frame = cvLoadImage(file_path);

            // find chessboard corners approximately
            found_flag = cvFindChessboardCorners(frame,		    // gray scale image
                                                 m_board_size,  // size of chessboard
                                                 corners,	    // array that will store corners
                                                 &corner_count, // corners count inside chessboard
                                                 CV_CALIB_CB_ADAPTIVE_THRESH + // adaptive threshold to convert the image to black and white
                                                 CV_CALIB_CB_FILTER_QUADS +	   // add contour area, perimeter, square-like shape etc.
                                                 CV_CALIB_CB_NORMALIZE_IMAGE   // normalize brightness and increase contrast
            ); // end of cvFindChessboardCorners function

            // draw found corners on frame and show frame
            cvDrawChessboardCorners(frame, m_board_size, corners, corner_count, found_flag);
            cvShowImage("Kalibrasyon", frame);
            cvWaitKey(1);

            // if found cornerCount equal real total corner count, enter if block
            // and store image points are found
            if(corner_count == point_count) {
                // convert color from RGBA to GRAY
                cvCvtColor(frame, gray_frame, CV_BGR2GRAY);

                // find corners sub pix, refine corner locations correctly
                cvFindCornerSubPix( gray_frame,     // gray scale image
                                    corners,        // array that store corners
                                    corner_count,   // corners count
                                    cvSize(11,11),  // half size of search window
                                    cvSize(-1,-1),  // half size of the dead region in the middle of the search zone
                                    // Criteria for termination of the iterative process of corner refinement.
                                    // That is, the process of corner position refinement stops either after criteria.maxCount
                                    // iterations or when the corner position moves by less than criteria.epsilon on some iteration.
                                    cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, FLT_EPSILON)
                ); // end of cvFindCornerSubPix function

                // fill found points to imageVector
                for(int i=0 ; i < point_count ; ++i) {
                    point.x = corners[i].x;
                    point.y = corners[i].y;
                    image_vector.push_back(point);
                } // end of for

                // increase frames count
                ++frame_count;
            } // end of if
        } // end of if
    } // end of while

    // destroy window
    cvDestroyAllWindows();
              
	// print video frames count
	cout << "Frames Count of CAM " << which_camera << " : " << frame_count << endl;

	// allocate memory for parameters of calibration function
	m_object_points = cvCreateMat(frame_count * point_count, 3, CV_64FC1);
	m_image_points  = cvCreateMat(frame_count * point_count, 2, CV_64FC1);
	m_found_points  = cvCreateMat(frame_count, 1, CV_32SC1);
	// rotation and translation vectors
	CvMat *rotation_vector    = cvCreateMat(frame_count, 3, CV_64FC1);
	CvMat *translation_vector = cvCreateMat(frame_count, 3, CV_64FC1);

	// fill image points and found corner number to imagePoints and foundPoint matrices
	for(int i=0 ; i < frame_count ; ++i) {
		for(int j=0 ; j < point_count ; ++j) {
			// fill image points
			CV_MAT_ELEM(*m_image_points, double, i*point_count+j, 0) = image_vector.at(i*point_count+j).x;
			CV_MAT_ELEM(*m_image_points, double, i*point_count+j, 1) = image_vector.at(i*point_count+j).y;
		}

		// fill found corners points count
		CV_MAT_ELEM(*m_found_points, int, i, 0) = point_count;
    }

	for(int i=0 ; i < frame_count ; ++i) {
		for(int j=0 ; j < m_row_count ; ++j) {
			for(int k=0 ; k < m_column_count ; ++k) {
				// fill object points
				// multiplication of index and squareSize is correct measurement
                CV_MAT_ELEM(*m_object_points, double, i*point_count + j*m_column_count + k, 0) = k*m_square_size;
                CV_MAT_ELEM(*m_object_points, double, i*point_count + j*m_column_count + k, 1) = j*m_square_size;
				// last column is zero because there is no depth
                CV_MAT_ELEM(*m_object_points, double, i*point_count + j*m_column_count + k, 2) = 0.0;
			}
		}
	}

    std::cout << "First calibration of CAM " << which_camera << " is being started ..." << std::endl;
    
	// calibrate camera. this calibration is temporary calibration to handle distortion coefficients
	error = cvCalibrateCamera2( m_object_points,	 // object points frame count x points on object x 3 or vice versa
								m_image_points,      // image points  frame count x found point count in every frame x 2 or vice versa
								m_found_points,      // found points  frame count x 1 or 1 x frame count
								m_image_size,		 // image size
								m_intrinsic_matrix,  // intrinsic matrix that contains of focal length, principal points and skew of CDDs 
								m_dis_coefficient,	 // distortion coefficients that may be 4x1, 5x1 or 8x1
								rotation_vector,	 // rotation vectors between coordinate systems of every frame and object
                                translation_vector	 // translation vectors between coordinate systems of every frame and object
                                //CV_CALIB_USE_INTRINSIC_GUESS
	); // end of calibration function

    std::cout << "First calibration of CAM " << which_camera << " is done ..." << std::endl;
    cvSave("distortion_coefficient_1.xml", m_dis_coefficient);

	// print first re-projection error
	std::cout << "First Re-projection Error of CAM " << which_camera << " : " << error / frame_count / point_count << std::endl; 
    
#if SECOND_CALIBRATION    
	// undistort image points with cvUndistortPoints function and
	// calculate new intrinsic matrix from undistorted points
	// create a destination matrices for distorted and undistorted points
	CvMat *distorted_image_points   = cvCreateMat(1, m_image_points->rows * m_image_points->cols, CV_64FC2);
	CvMat *undistorted_image_points = cvCreateMat(1, m_image_points->rows * m_image_points->cols, CV_64FC2);

	// fill 1-channel matrix with 2-channels matrix
	for(int i=0 ; i < m_image_points->rows ; ++i) {
		CV_MAT_ELEM(*distorted_image_points, double, 0, i*m_image_points->cols) = CV_MAT_ELEM(*m_image_points, double, i, 0);
		CV_MAT_ELEM(*distorted_image_points, double, 0, i*m_image_points->cols + 1) = CV_MAT_ELEM(*m_image_points, double, i, 1);
	}

	// undistort image points to make calibration correctly
	cvUndistortPoints(distorted_image_points, undistorted_image_points, m_intrinsic_matrix, m_dis_coefficient);

	// fill 2-channels matrix with 1-channel matrix
    // copy undistorted image points matrix to image points matrix
	for(int i=0 ; i < m_image_points->rows ; ++i) {
		CV_MAT_ELEM(*m_image_points, double, i, 0) = CV_MAT_ELEM(*undistorted_image_points, double, 0, i*m_image_points->cols);
		CV_MAT_ELEM(*m_image_points, double, i, 1) = CV_MAT_ELEM(*undistorted_image_points, double, 0, i*m_image_points->cols + 1);
	}

    std::cout << "Second calibration of CAM " << which_camera << " is being started ..." << std::endl;
    
	// run calibration function with undistorted image points to calibrate camera correctly
	error = cvCalibrateCamera2( m_object_points,	// object points frame count x points on object x 3 or vice versa
								m_image_points,	    // image points  frame count x found point count in every frame x 2 or vice versa
								m_found_points,	    // found points  frame count x 1 or 1 x frame count
								m_image_size,		// image size
								m_intrinsic_matrix, // intrinsic matrix that contains of focal length, principal points and skew of CDDs 
								m_dis_coefficient,  // distortion coefficients that may be 4x1, 5x1 or 8x1
								rotation_vector,    // rotation vectors between coordinate systems of every frame and object
								translation_vector, // translation vectors between coordinate systems of every frame and object
                                CV_CALIB_USE_INTRINSIC_GUESS
	); // end of calibration function

    std::cout << "Second calibration of CAM " << which_camera << " is done ..." << std::endl;
	// print first re-projection error after undistortion of points
	std::cout << "Second Re-projection Error of CAM " << which_camera << " : " << error / frame_count / point_count << std::endl;
#endif
    
    // estimate extrinsic parameters
    cvFindExtrinsicCameraParams2(m_object_points, m_image_points, m_intrinsic_matrix, m_dis_coefficient, m_rotation_vector, m_translation_vector);
    cvRodrigues2(m_rotation_vector, m_rotation_matrix);

    // estimate projection matrix
    for(int i=0 ; i < 3 ; ++i) {
        for(int j=0 ; j < 4 ; ++j) {
            if(j == 3)
                CV_MAT_ELEM(*m_projection_matrix, double, i, j) = CV_MAT_ELEM(*m_translation_vector, double, i, 0);
            else
                CV_MAT_ELEM(*m_projection_matrix, double, i, j) = CV_MAT_ELEM(*m_rotation_matrix, double, i, j);
        }
    }

    cvMatMul(m_intrinsic_matrix, m_projection_matrix, m_projection_matrix);

    // save matrices and vectors informations in XML files for first cam
    cvSave("object_points.xml", m_object_points);
    cvSave("image_points.xml", m_image_points);
    cvSave("intrinsic_matrix.xml", m_intrinsic_matrix);
    cvSave("rotation_matrix.xml", m_rotation_matrix);
    cvSave("translation_vector.xml", m_translation_vector);
    cvSave("projection_matrix.xml", m_projection_matrix);
    
#if SECOND_CALIBRATION    
	cvReleaseMat(&distorted_image_points);
	cvReleaseMat(&undistorted_image_points);
#endif
    
    // release matrices
    cvReleaseMat(&rotation_vector);
    cvReleaseMat(&translation_vector);
	// release image
	cvReleaseImage(&frame);
	cvReleaseImage(&gray_frame);
	// release corners
	delete [] corners;
    // close directory
    closedir(dir);

	return;
}
