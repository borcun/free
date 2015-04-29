#include "ABFundamentalMatrix.hpp"

// default constructor
ABFundamentalMatrix::ABFundamentalMatrix()
: m_error(1.0)
{
	// allocate memory for fundamental matrix
	m_fundamental_matrix = cvCreateMat(3, 3, CV_64FC1);
	m_u_mat  = cvCreateMat(3, 3, CV_64FC1);
	m_d_mat  = cvCreateMat(3, 3, CV_64FC1);
	m_vt_mat = cvCreateMat(3, 3, CV_64FC1);
}

// constructor
ABFundamentalMatrix::ABFundamentalMatrix(vector<CvPoint2D64f> left_points, vector<CvPoint2D64f> right_points)
: m_error(1.0)
{
	// allocate memory for fundamental matrix
	m_fundamental_matrix = cvCreateMat(3, 3, CV_64FC1);
	m_u_mat  = cvCreateMat(3, 3, CV_64FC1);
	m_d_mat  = cvCreateMat(3, 3, CV_64FC1);
	m_vt_mat = cvCreateMat(3, 3, CV_64FC1);
	// call setImagePointVectors function to set vectors
	setImagePointVectors(left_points, right_points);
}

// destructor
ABFundamentalMatrix::~ABFundamentalMatrix()
{
	// clear vectors
	m_left_image_points.clear();
	m_right_image_points.clear();
	// release fundamental and its component matrices
	cvReleaseMat(&m_fundamental_matrix);
	cvReleaseMat(&m_u_mat);
	cvReleaseMat(&m_d_mat);
	cvReleaseMat(&m_vt_mat);
}

// function that sets vectors of left and right images 64-bit points
void ABFundamentalMatrix::setImagePointVectors(vector<CvPoint2D64f> left_points, vector<CvPoint2D64f> right_points)
{
	// copy feature points vectors to left and right image vector
	// their sizes are same, it can be in just one for loop
	for(int i=0 ; i < (int)left_points.size() ; ++i) {
		m_left_image_points.push_back(left_points.at(i));
	} // end of for

	for(int i=0 ; i < (int)right_points.size() ; ++i) {
		m_right_image_points.push_back(right_points.at(i));
	} // end of for
}

// function that finds fundamental matrix and return it
CvMat *ABFundamentalMatrix::findFundamentalMat(const double correstness)
{
	int index = 0;

	// create left and right image matrices and fundamental matrix
	CvMat *left_image_matrix  = cvCreateMat(3, m_left_image_points.size(),  CV_64FC1);
	CvMat *right_image_matrix = cvCreateMat(3, m_right_image_points.size(), CV_64FC1);

	// if we don't expand matrix by 1 to make 3xN matrix, 
	// cvFindFundamentalMat function expands them
	for(int i=0 ; i < (int)m_left_image_points.size() ; ++i) {
		CV_MAT_ELEM(*left_image_matrix, double, 0, i) = m_left_image_points.at(i).x;
		CV_MAT_ELEM(*left_image_matrix, double, 1, i) = m_left_image_points.at(i).y;
		CV_MAT_ELEM(*left_image_matrix, double, 2, i) = 1;
	} // end of for

	for(int i=0 ; i < (int)m_right_image_points.size() ; ++i) {
		CV_MAT_ELEM(*right_image_matrix, double, 0, i) = m_right_image_points.at(i).x;
		CV_MAT_ELEM(*right_image_matrix, double, 1, i) = m_right_image_points.at(i).y;
		CV_MAT_ELEM(*right_image_matrix, double, 2, i) = 1;
	} // end of for

	// find fundamental matrix by using CV_RANSAC method
	// CV_FM_RANSAC parameter calls runRANSAC function of CvModelEstimator2 class
	// source file : modules/calib3d/src/modelest.cpp
	// 5. parameter used for RANSAC. It is the maximum distance from a point to an epipolar line in pixels
	// 6. parameter used for the RANSAC or LMedS methods only. 
	// It specifies a desirable level of confidence (probability) that the estimated matrix is correct.
	for(int i=0 ; i < 30 ; ++i) {
		cvFindFundamentalMat(left_image_matrix, right_image_matrix, m_fundamental_matrix, CV_FM_RANSAC, MAX_THRESHOLD - (double)i / 10 * MIN_THRESHOLD, correstness);
		
		// call decomposeFundamentalMat function to checks fundamental correction
		decomposeFundamentalMat();

		// find index that gives minimum error in estimation of fundamental matrix
		if(m_error > fabs(CV_MAT_ELEM(*m_d_mat, double, 2, 2))) {
			m_error = fabs(CV_MAT_ELEM(*m_d_mat, double, 2, 2));
			index = i;
		}
	}

	// find fundamental matrix again with minimum error
	cvFindFundamentalMat(left_image_matrix, right_image_matrix, m_fundamental_matrix, CV_FM_RANSAC, MAX_THRESHOLD - (double)index / 10 * MIN_THRESHOLD, correstness);
	// refine fundamental matrix
    //refineFundamentalMatrix();
	// call decomposeFundamentalMat function to checks fundamental correction
	decomposeFundamentalMat();
	// set err
	m_error = CV_MAT_ELEM(*m_d_mat, double, 2, 2);
	cout << endl << "Error of Estimation Fundamental Matrix : " << m_error << endl;

	// release matrices
	cvReleaseMat(&left_image_matrix);
	cvReleaseMat(&right_image_matrix);

	// return fundamental matrix
	return m_fundamental_matrix;
}

// function that decomposes fundamental matrix by using SVD method
void ABFundamentalMatrix::decomposeFundamentalMat()
{
	// compute decomposition of fundamental matrix
	// F = U . D . vT
	// cvSVD returns matrices of fundamental matrix by D, U, vT
	cvSVD(m_fundamental_matrix, m_d_mat, m_u_mat, m_vt_mat, cv::SVD::FULL_UV);
	
	return;
}

// function that refines fundamental matrix with using Levenberg-Marquardt algorithm
void ABFundamentalMatrix::refineFundamentalMatrix()
{
	ABLevenbergMarquardt *levmar = new ABLevenbergMarquardt();
	CvMat *row_fun_mat = cvCreateMat(m_fundamental_matrix->rows * m_fundamental_matrix->cols, 1, m_fundamental_matrix->type);
	CvTermCriteria criteria;

	// convert 3x3 fundamental matrix to 9x1 row matrix
	for(int i=0 ; i < 3 ; ++i) {
		for(int j=0 ; j < 3 ; ++j) {
			CV_MAT_ELEM(*row_fun_mat, double, i*3+j, 0) = CV_MAT_ELEM(*m_fundamental_matrix, double, i, j);
		}
	}

	// set criteria
	criteria.max_iter = 100;
	criteria.epsilon = DBL_EPSILON;

	// create Jacobian and error matrices
	levmar->createJacobianMatrix(m_left_image_points, m_right_image_points, m_fundamental_matrix);
	// call run function and optimize fundamental matrix
	CvMat *refine_fun_mat = levmar->run(levmar->getJacobianMatrix(), levmar->getErrorMatrix(), row_fun_mat, m_left_image_points, m_right_image_points, criteria);

	// convert optimized 9x1 row matrix to 3x3 fundamental matrix
	for(int i=0 ; i < 3 ; ++i) {
		for(int j=0 ; j < 3 ; ++j) {
			CV_MAT_ELEM(*m_fundamental_matrix, double, i, j) = CV_MAT_ELEM(*refine_fun_mat, double, i*3+j, 0);
		}
	}

	// release matrices
	cvReleaseMat(&row_fun_mat);

	return;
}
