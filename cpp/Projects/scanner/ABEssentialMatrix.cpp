#include "ABEssentialMatrix.hpp"

// default constructor
ABEssentialMatrix::ABEssentialMatrix()
: m_error(-1.0)
{
	// allocate memory for essential matrix
	m_essential_matrix   = cvCreateMat(3, 3, CV_64FC1);
	m_fundamental_matrix = cvCreateMat(3, 3, CV_64FC1);
	m_intrinsic_matrix_1  = cvCreateMat(3, 3, CV_64FC1);
	m_intrinsic_matrix_2  = cvCreateMat(3, 3, CV_64FC1);
	m_intrinsic_matrix_inv_1  = cvCreateMat(3, 3, CV_64FC1);
	m_intrinsic_matrix_inv_2  = cvCreateMat(3, 3, CV_64FC1);
	m_rotation_matrix_1    = cvCreateMat(3, 3, CV_64FC1);
	m_rotation_matrix_2    = cvCreateMat(3, 3, CV_64FC1);
	m_translation_matrix_1 = cvCreateMat(3, 1, CV_64FC1);
	m_translation_matrix_2 = cvCreateMat(3, 1, CV_64FC1);
	m_u_mat  = cvCreateMat(3, 3, CV_64FC1);
	m_d_mat  = cvCreateMat(3, 3, CV_64FC1);
	m_vt_mat = cvCreateMat(3, 3, CV_64FC1);
}

// constructor
ABEssentialMatrix::ABEssentialMatrix(const CvMat *intrinsic_matrix_1, const CvMat *intrinsic_matrix_2, const CvMat *fundamental_matrix)
: m_error(-1.0)
{
	// allocate memory for essential matrix
	m_essential_matrix   = cvCreateMat(3, 3, CV_64FC1);
	m_fundamental_matrix = cvCreateMat(3, 3, CV_64FC1);
	m_intrinsic_matrix_1  = cvCreateMat(3, 3, CV_64FC1);
	m_intrinsic_matrix_2  = cvCreateMat(3, 3, CV_64FC1);
	m_intrinsic_matrix_inv_1 = cvCreateMat(3, 3, CV_64FC1);
	m_intrinsic_matrix_inv_2 = cvCreateMat(3, 3, CV_64FC1);
	m_rotation_matrix_1 = cvCreateMat(3, 3, CV_64FC1);
	m_rotation_matrix_2 = cvCreateMat(3, 3, CV_64FC1);
	m_translation_matrix_1 = cvCreateMat(3, 1, CV_64FC1);
	m_translation_matrix_2 = cvCreateMat(3, 1, CV_64FC1);
	m_u_mat  = cvCreateMat(3, 3, CV_64FC1);
	m_d_mat  = cvCreateMat(3, 3, CV_64FC1);
	m_vt_mat = cvCreateMat(3, 3, CV_64FC1);
	// set intrinsic matrix and fundamental matrix
	setIntrinsicMatrices(intrinsic_matrix_1, intrinsic_matrix_2);
	setFundamentalMatrix(fundamental_matrix);
}

// constructor
ABEssentialMatrix::ABEssentialMatrix(vector<CvPoint2D64f> left_points, vector<CvPoint2D64f> right_points, const CvMat *intrinsic_matrix_1, const CvMat *intrinsic_matrix_2, const CvMat *fundamental_matrix)
: m_error(-1.0)
{
	// allocate memory for essential matrix
	m_essential_matrix   = cvCreateMat(3, 3, CV_64FC1);
	m_fundamental_matrix = cvCreateMat(3, 3, CV_64FC1);
	m_intrinsic_matrix_1  = cvCreateMat(3, 3, CV_64FC1);
	m_intrinsic_matrix_2  = cvCreateMat(3, 3, CV_64FC1);    
	m_intrinsic_matrix_inv_1 = cvCreateMat(3, 3, CV_64FC1);
	m_intrinsic_matrix_inv_2 = cvCreateMat(3, 3, CV_64FC1);
	m_rotation_matrix_1 = cvCreateMat(3, 3, CV_64FC1);
	m_rotation_matrix_2 = cvCreateMat(3, 3, CV_64FC1);
	m_translation_matrix_1 = cvCreateMat(3, 1, CV_64FC1);
	m_translation_matrix_2 = cvCreateMat(3, 1, CV_64FC1);
	m_u_mat  = cvCreateMat(3, 3, CV_64FC1);
	m_d_mat  = cvCreateMat(3, 3, CV_64FC1);
	m_vt_mat = cvCreateMat(3, 3, CV_64FC1);
	// set intrinsic matrix and fundamental matrix
	setIntrinsicMatrices(intrinsic_matrix_1, intrinsic_matrix_2);
	setFundamentalMatrix(fundamental_matrix);
    // set image points
    setImagePointVectors(left_points, right_points);
}

// destructor
ABEssentialMatrix::~ABEssentialMatrix()
{
	// release matrices
	cvReleaseMat(&m_essential_matrix);
	cvReleaseMat(&m_fundamental_matrix);
	cvReleaseMat(&m_intrinsic_matrix_1);
	cvReleaseMat(&m_intrinsic_matrix_2);
	cvReleaseMat(&m_intrinsic_matrix_inv_1);
	cvReleaseMat(&m_intrinsic_matrix_inv_2);
	cvReleaseMat(&m_rotation_matrix_1);
	cvReleaseMat(&m_rotation_matrix_2);
	cvReleaseMat(&m_translation_matrix_1);
	cvReleaseMat(&m_translation_matrix_2);
	cvReleaseMat(&m_u_mat);
	cvReleaseMat(&m_d_mat);
	cvReleaseMat(&m_vt_mat);
    // clear vectors
    m_right_image_points.clear();
    m_left_image_points.clear();
}

// function that sets intrinsic matrices
void ABEssentialMatrix::setIntrinsicMatrices(const CvMat *intrinsic_matrix_1, const CvMat *intrinsic_matrix_2)
{
    // copy intrinsic matrices to data members
    cvCopy(intrinsic_matrix_1, m_intrinsic_matrix_1);
    cvCopy(intrinsic_matrix_2, m_intrinsic_matrix_2);

	return;
}

// function that sets fundamental matrix
void ABEssentialMatrix::setFundamentalMatrix(const CvMat *fundamental_matrix)
{
	cvCopy(fundamental_matrix, m_fundamental_matrix);
    
    return;
}

// function that sets vectors of left and right images 64-bit points
void ABEssentialMatrix::setImagePointVectors(vector<CvPoint2D64f> left_points, vector<CvPoint2D64f> right_points)
{
	// copy feature points vectors to left and right image vector
	// their sizes are same, it can be in just one for loop
	for(int i=0 ; i < (int)left_points.size() ; ++i) {
		m_left_image_points.push_back(left_points.at(i));
	} // end of for

	for(int i=0 ; i < (int)right_points.size() ; ++i) {
		m_right_image_points.push_back(right_points.at(i));
	} // end of for
    
    return;
}

// function that finds essential matrix
CvMat *ABEssentialMatrix::findEssentialMat()
{
	// Essential matrix can found by formula that is K2^ . F . K1
	CvMat *intrinsic_mat2_trans = cvCreateMat(3, 3, CV_64FC1);
	
	// find transpose of intrinsic matrix
	cvTranspose(m_intrinsic_matrix_2, intrinsic_mat2_trans);

	// find multiplication of transpose of intrinsic and fundamental matrix
	cvMatMul(intrinsic_mat2_trans, m_fundamental_matrix, m_essential_matrix);
	// find multiplication of holdResult and intrinsic matrix
	cvMatMul(m_essential_matrix, m_intrinsic_matrix_1, m_essential_matrix);

	// refine essential matrix
	//refineEssentialMatrix();

	// call decomposeEssentialMat function to find error when essential matrix
	decomposeEssentialMat();

	m_error = CV_MAT_ELEM(*m_d_mat, double, 2, 2);
	cout << endl << "Error of Estimation Essential Matrix : " << m_error << endl;
	cout << "Determinant of Essential Matrix : " << cvDet(m_essential_matrix) << endl;   
    
	// release matrix
	cvReleaseMat(&intrinsic_mat2_trans);

	// return essential matrix
	return m_essential_matrix;
}

// functions that get first rotation matrix
CvMat *ABEssentialMatrix::getRotationMatrix1() const 
{ 
	return m_rotation_matrix_1; 
}

// functions that get second rotation matrix
CvMat *ABEssentialMatrix::getRotationMatrix2() const 
{ 
	return m_rotation_matrix_2; 
}

// functions that get first translation matrix
CvMat *ABEssentialMatrix::getTranslationMatrix1() const 
{ 
	return m_translation_matrix_1; 
}

// functions that get second translation matrix
CvMat *ABEssentialMatrix::getTranslationMatrix2() const 
{ 
	return m_translation_matrix_2; 
}

// function that gets error of SVD process
double ABEssentialMatrix::getError() const 
{ 
	return m_error; 
}

// function that decomposes essential matrix and fill rotation and translation matrices
// The formulas are from Multiple View Geometry [ A. Zisserman and R. Hartley ]
void ABEssentialMatrix::decomposeEssentialMat()
{
	// compute decomposition of essential matrix
	// E = U . D . vT
	// cvSVD returns matrices of fundamental matrix by D, U, vT
	cvSVD(m_essential_matrix, m_d_mat, m_u_mat, m_vt_mat, cv::SVD::FULL_UV);

	/* After decomposition of essential matrix, rotation and translation matrices
	 * can be found but a necessary matrix for finding rotation has different contents.
	 *
	 *						|  0  1  0 |						| 0 -1  0 |
	 *		(Z) Matrix =	| -1  0  0 |  ,		(W) Matrix =	| 1  0  0 |
	 *						|  0  0  0 |						| 0  0  1 |
	 *
	 * Z Matrix skew-symmetric matrix, W matrix is orthogonal matrix.
	 * Cause of different matrices, results can be different. Results are symmetric.
	 * Rotation matrix is (U . w . vT) or (U . wT . vT)
	 * Translation matrix is last column of uMat or negative of last column
	 * Because translation matrix is dependent of R, it changes like rotation matrix.
	 */
	CvMat *z_mat = cvCreateMat(3, 3, CV_64FC1);
	CvMat *w_mat = cvCreateMat(3, 3, CV_64FC1);
	CvMat *w_mat_transpose = cvCreateMat(3, 3, CV_64FC1);

	// zero matrices
	cvSetZero(w_mat);
	cvSetZero(z_mat);

	// add orthogonal values to wMat
	CV_MAT_ELEM(*w_mat, double, 0, 1) = -1.0;
	CV_MAT_ELEM(*w_mat, double, 1, 0) =  1.0;
	CV_MAT_ELEM(*w_mat, double, 2, 2) =  1.0;
	// add skew-symmetric parameters to zMat
	CV_MAT_ELEM(*z_mat, double, 0, 1) =  1.0;
	CV_MAT_ELEM(*z_mat, double, 1, 0) = -1.0;

	// find transpose of wMat
	cvTranspose(w_mat, w_mat_transpose);

	// calculate possible rotation matrices
	cvMatMul(m_u_mat, w_mat, m_rotation_matrix_1);
	cvMatMul(m_u_mat, w_mat_transpose, m_rotation_matrix_2);
	cvMatMul(m_rotation_matrix_1, m_vt_mat, m_rotation_matrix_1);
	cvMatMul(m_rotation_matrix_2, m_vt_mat, m_rotation_matrix_2);
	
	// set translation matrices
	CV_MAT_ELEM(*m_translation_matrix_1, double, 0, 0) = CV_MAT_ELEM(*m_u_mat, double, 0, 2);
	CV_MAT_ELEM(*m_translation_matrix_1, double, 1, 0) = CV_MAT_ELEM(*m_u_mat, double, 1, 2);
	CV_MAT_ELEM(*m_translation_matrix_1, double, 2, 0) = CV_MAT_ELEM(*m_u_mat, double, 2, 2);
	CV_MAT_ELEM(*m_translation_matrix_2, double, 0, 0) = -1*CV_MAT_ELEM(*m_u_mat, double, 0, 2);
	CV_MAT_ELEM(*m_translation_matrix_2, double, 1, 0) = -1*CV_MAT_ELEM(*m_u_mat, double, 1, 2);
	CV_MAT_ELEM(*m_translation_matrix_2, double, 2, 0) = -1*CV_MAT_ELEM(*m_u_mat, double, 2, 2);

	// release matrices
	cvReleaseMat(&z_mat);
	cvReleaseMat(&w_mat);
	cvReleaseMat(&w_mat_transpose);

	return;
}

// function that refines essential matrix by LevMar algorithm
void ABEssentialMatrix::refineEssentialMatrix()
{
	//ABLevenbergMarquardt *levmar = new ABLevenbergMarquardt();
	//// normalized image points
	//vector<CvPoint3D64f> nor_left_points;
	//vector<CvPoint3D64f> nor_right_points;
	//CvPoint3D64f point;
	//CvTermCriteria criteria;
	//CvMat *row_ess_mat = cvCreateMat(m_essential_matrix->rows * m_essential_matrix->cols, 1, m_essential_matrix->type);

	//// convert 3x3 essential matrix to 9x1 row matrix
	//for(int i=0 ; i < 3 ; ++i) {
	//	for(int j=0 ; j < 3 ; ++j) {
	//		CV_MAT_ELEM(*row_ess_mat, double, i*3+j, 0) = CV_MAT_ELEM(*m_essential_matrix, double, i, j);
	//	}
	//}

	//// set criteria
	//criteria.max_iter = 100;
	//criteria.epsilon = DBL_EPSILON;

	//// normalized image points to process essential matrix formula
	//// (normalized left image points)T * Essential Matrix * (normalized right image points) = 0
	//for(int i=0 ; i < (int)m_left_image_points.size() ; ++i) {
	//	// fill nor_left_points
	//	point.x = CV_MAT_ELEM(*m_intrinsic_matrix_inv_1, double, 0, 0) * m_left_image_points.at(i).x + 
	//			  CV_MAT_ELEM(*m_intrinsic_matrix_inv_1, double, 0, 1) * m_left_image_points.at(i).y + 
	//			  CV_MAT_ELEM(*m_intrinsic_matrix_inv_1, double, 0, 2); 
	//	point.y = CV_MAT_ELEM(*m_intrinsic_matrix_inv_1, double, 1, 0) * m_left_image_points.at(i).x + 
	//			  CV_MAT_ELEM(*m_intrinsic_matrix_inv_1, double, 1, 1) * m_left_image_points.at(i).y + 
	//		      CV_MAT_ELEM(*m_intrinsic_matrix_inv_1, double, 1, 2); 
	//	point.z = CV_MAT_ELEM(*m_intrinsic_matrix_inv_1, double, 2, 0) * m_left_image_points.at(i).x + 
	//			  CV_MAT_ELEM(*m_intrinsic_matrix_inv_1, double, 2, 1) * m_left_image_points.at(i).y + 
	//			  CV_MAT_ELEM(*m_intrinsic_matrix_inv_1, double, 2, 2); 
	//	// add point to n_left_point
	//	nor_left_points.push_back(point);

	//	// fill nor_right_points
	//	point.x = CV_MAT_ELEM(*m_intrinsic_matrix_inv_2, double, 0, 0) * m_right_image_points.at(i).x + 
	//			  CV_MAT_ELEM(*m_intrinsic_matrix_inv_2, double, 0, 1) * m_right_image_points.at(i).y + 
	//			  CV_MAT_ELEM(*m_intrinsic_matrix_inv_2, double, 0, 2); 
	//	point.y = CV_MAT_ELEM(*m_intrinsic_matrix_inv_2, double, 1, 0) * m_right_image_points.at(i).x + 
	//			  CV_MAT_ELEM(*m_intrinsic_matrix_inv_2, double, 1, 1) * m_right_image_points.at(i).y + 
	//			  CV_MAT_ELEM(*m_intrinsic_matrix_inv_2, double, 1, 2); 
	//	point.z = CV_MAT_ELEM(*m_intrinsic_matrix_inv_2, double, 2, 0) * m_right_image_points.at(i).x + 
	//			  CV_MAT_ELEM(*m_intrinsic_matrix_inv_2, double, 2, 1) * m_right_image_points.at(i).y + 
	//			  CV_MAT_ELEM(*m_intrinsic_matrix_inv_2, double, 2, 2); 
	//	// add point to n_left_point
	//	nor_right_points.push_back(point);

	//}

	//// create Jacobian and error matrices
	//levmar->createJacobianMatrix(nor_left_points, nor_right_points, m_essential_matrix);
	//// call run function and optimize essential matrix
	//CvMat *refine_ess_mat = levmar->run(levmar->getJacobianMatrix(), levmar->getErrorMatrix(), row_ess_mat, nor_left_points, nor_right_points, criteria);

	//// convert optimized 9x1 row matrix to 3x3 essential matrix
	//for(int i=0 ; i < 3 ; ++i) {
	//	for(int j=0 ; j < 3 ; ++j) {
	//		CV_MAT_ELEM(*m_essential_matrix, double, i, j) = CV_MAT_ELEM(*refine_ess_mat, double, i*3+j, 0);
	//	}
	//}

	//// release matrices
	//cvReleaseMat(&row_ess_mat);

	return;
}