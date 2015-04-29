#include "ABReconstruction.hpp"

// default constructor
ABReconstruction::ABReconstruction()
{
	// allocate memory for matrices
	m_intrinsic_matrix1  = cvCreateMat(3, 3, CV_64FC1);
	m_intrinsic_matrix2  = cvCreateMat(3, 3, CV_64FC1);
	m_rotation_matrix	 = cvCreateMat(3, 3, CV_64FC1);
	m_translation_matrix = cvCreateMat(3, 1, CV_64FC1);
	m_projection_matrix1 = cvCreateMat(3, 4, CV_64FC1);
	m_projection_matrix2 = cvCreateMat(3, 4, CV_64FC1);
}

// constructor
ABReconstruction::ABReconstruction(const CvMat *intrinsic_matrix1, const CvMat *intrinsic_matrix2, const CvMat *rotation_matrix, const CvMat *translation_matrix)
{
	// allocate memory for matrices
	m_intrinsic_matrix1  =  cvCreateMat(3, 3, CV_64FC1);
	m_intrinsic_matrix2  = cvCreateMat(3, 3, CV_64FC1);
	m_rotation_matrix	 = cvCreateMat(3, 3, CV_64FC1);
	m_translation_matrix = cvCreateMat(3, 1, CV_64FC1);
	m_projection_matrix1 = cvCreateMat(3, 4, CV_64FC1);
	m_projection_matrix2 = cvCreateMat(3, 4, CV_64FC1);
	// set matrices
	setIntrinsicMatrices(intrinsic_matrix1, intrinsic_matrix2);
	setRotationMatrix(rotation_matrix);
	setTranslationMatrix(translation_matrix);
}

// destructor
ABReconstruction::~ABReconstruction()
{
	// release matrices
	cvReleaseMat(&m_intrinsic_matrix1);
	cvReleaseMat(&m_intrinsic_matrix2);
	cvReleaseMat(&m_rotation_matrix);
	cvReleaseMat(&m_translation_matrix);
	cvReleaseMat(&m_projection_matrix1);
	cvReleaseMat(&m_projection_matrix2);
	cvReleaseMat(&m_left_image_points);
	cvReleaseMat(&m_right_image_points);
	cvReleaseMat(&m_left_image_points1D);
	cvReleaseMat(&m_right_image_points1D);
	cvReleaseMat(&m_correct_left_image_points);
	cvReleaseMat(&m_correct_right_image_points);
	cvReleaseMat(&m_world_points);
}

// function that sets intrinsic matrices
void ABReconstruction::setIntrinsicMatrices(const CvMat *intrinsic_matrix1, const CvMat *intrinsic_matrix2)
{
	cvCopy(intrinsic_matrix1, m_intrinsic_matrix1);
	cvCopy(intrinsic_matrix2, m_intrinsic_matrix2);
}

// function that sets rotation matrix
void ABReconstruction::setRotationMatrix(const CvMat *rotation_matrix)
{
	cvCopy(rotation_matrix, m_rotation_matrix);
}

// function that sets translation matrix
void ABReconstruction::setTranslationMatrix(const CvMat *translation_matrix)
{
	cvCopy(translation_matrix, m_translation_matrix);
}

// function that set first projection matrix
CvMat *ABReconstruction::getFirstProjectionMatrix()
{
	// zero first projection matrix 
	// function zeros all elements of matrix
	cvSetZero(m_projection_matrix1);

	// fill projection matrix 1
	// first 3 rows and 3 columns are intrinsic matrix values
	// last column is all zero
	for(int i=0 ; i < 3 ; ++i) {
		for(int j=0 ; j < 3 ; ++j) {
			CV_MAT_ELEM(*m_projection_matrix1, double, i, j) = CV_MAT_ELEM(*m_intrinsic_matrix1, double, i, j);
		}
	}

	return m_projection_matrix1;
}

// function that set second projection matrix
CvMat *ABReconstruction::getSecondProjectionMatrix()
{
	// KR matrix is multiplication of intrinsic matrix and rotation matrix
	CvMat *KR = cvCreateMat(3, 3, CV_64FC1);
	// KT matrix is multiplication of intrinsic matrix and translation matrix
	CvMat *KT = cvCreateMat(3, 1, CV_64FC1);

	// find multiplication of intrinsic matrix and rotation matrix
	cvMatMul(m_intrinsic_matrix2, m_rotation_matrix, KR);
	// find multiplication of intrinsic matrix and translation matrix
	cvMatMul(m_intrinsic_matrix2, m_translation_matrix, KT);

	// fill projection matrix 2
	for(int i=0 ; i < 3 ; ++i) {
		for(int j=0 ; j < 4 ; ++j) {
			if(j < 3)
				CV_MAT_ELEM(*m_projection_matrix2, double, i, j) = CV_MAT_ELEM(*KR, double, i, j);
			else
				CV_MAT_ELEM(*m_projection_matrix2, double, i, j) = CV_MAT_ELEM(*KT, double, i, 0);
		}
	}

	// release matrices
	cvReleaseMat(&KR);
	cvReleaseMat(&KT);
	
	return m_projection_matrix2;
}

// function that triangulates corresponded points and fill world points matrix
void ABReconstruction::triangulatePoints(vector<CvPoint2D64f> left_image_vector, vector<CvPoint2D64f> right_image_vector, CvMat *fundamental_matrix)
{
	// check sizes of corresponded points vectors
	if(left_image_vector.size() != right_image_vector.size()) {
		cerr << "CORRESPONDED POINTS COUNT MUST BE SAME" << endl;
		return ;
	}

	// get size of vectors
	const int size = left_image_vector.size();

	// allocate memory for matrices
	m_left_image_points    = cvCreateMat(1, size * 2,  CV_64FC2);
	m_right_image_points   = cvCreateMat(1, size * 2, CV_64FC2);
	m_left_image_points1D  = cvCreateMat(1, size * 2,  CV_64FC2);
	m_right_image_points1D = cvCreateMat(1, size * 2, CV_64FC2);
	m_correct_left_image_points  = cvCreateMat(2, size,  CV_64FC1);
	m_correct_right_image_points = cvCreateMat(2, size, CV_64FC1);
	m_world_points = cvCreateMat(4, size, CV_64FC1);

	// copy points from vectors to matrices by converting from 2D to 1D
	for(int i=0 ; i < size ; ++i) {
		// fill left image points
		CV_MAT_ELEM(*m_left_image_points, double, 0, i*2) = left_image_vector.at(i).x;
		CV_MAT_ELEM(*m_left_image_points, double, 0, i*2+1) = left_image_vector.at(i).y;
		// fill right image points
		CV_MAT_ELEM(*m_right_image_points, double, 0, i*2) = right_image_vector.at(i).x;
		CV_MAT_ELEM(*m_right_image_points, double, 0, i*2+1) = right_image_vector.at(i).y;
	}

	// corresponded points are calculated again for finding robust correspondence with using
	// minimization criteria d(u , ~u) ^ 2 + d(u' , ~u') ^ 2 where d(a,b) represents Euclidean distance
	cvCorrectMatches(fundamental_matrix, m_left_image_points, m_right_image_points, m_left_image_points1D, m_right_image_points1D);

	// copy 1D points to 2D points
	for(int i=0 ; i < size ; ++i) {
		// fill left image points
		CV_MAT_ELEM(*m_correct_left_image_points, double, 0, i) = CV_MAT_ELEM(*m_left_image_points1D, double, 0, i*2);
		CV_MAT_ELEM(*m_correct_left_image_points, double, 1, i) = CV_MAT_ELEM(*m_left_image_points1D, double, 0, i*2+1);
		// fill right image points
		CV_MAT_ELEM(*m_correct_right_image_points, double, 0, i) = CV_MAT_ELEM(*m_right_image_points1D, double, 0, i*2); 
		CV_MAT_ELEM(*m_correct_right_image_points, double, 1, i) = CV_MAT_ELEM(*m_right_image_points1D, double, 0, i*2+1);
	}

	// call triangulation function
	triangulate(m_projection_matrix1, m_projection_matrix2, m_correct_left_image_points, m_correct_right_image_points, m_world_points);

	return;
}

// function that gets world coordinate points
CvMat *ABReconstruction::getWorldPoints() const 
{ 
	return m_world_points; 
}

// function that triangulates corresponded points
void ABReconstruction::triangulate(CvMat* projMatr1, CvMat* projMatr2, CvMat* projPoints1, CvMat* projPoints2, CvMat* points4D)
{
	if(projMatr1 == 0 || projMatr2 == 0 || projPoints1 == 0 || projPoints2 == 0 || points4D == 0)
		CV_Error( CV_StsNullPtr, "Some of parameters is a NULL pointer" );

	if(!CV_IS_MAT(projMatr1) || !CV_IS_MAT(projMatr2) || !CV_IS_MAT(projPoints1) || !CV_IS_MAT(projPoints2) || !CV_IS_MAT(points4D))
		CV_Error( CV_StsUnsupportedFormat, "Input parameters must be matrices");

	int numPoints;
	numPoints = projPoints1->cols;

	if(numPoints < 1)
		CV_Error( CV_StsOutOfRange, "Number of points must be more than zero");

	if(projPoints2->cols != numPoints || points4D->cols != numPoints)
		CV_Error( CV_StsUnmatchedSizes, "Number of points must be the same" );

	if(projPoints1->rows != 2 || projPoints2->rows != 2)
		CV_Error( CV_StsUnmatchedSizes, "Number of projected points coordinates must be == 2");

	if(points4D->rows != 4 )
		CV_Error( CV_StsUnmatchedSizes, "Number of world points coordinates must be == 4" );

	if(projMatr1->cols != 4 || projMatr1->rows != 3 || projMatr2->cols != 4 || projMatr2->rows != 3)
		CV_Error( CV_StsUnmatchedSizes, "Size of projection matrices must be 3x4" );

	CvMat matrA;
	double matrA_dat[24];
	matrA = cvMat(6,4,CV_64FC1,matrA_dat);

	//CvMat matrU;
	CvMat matrW;
	CvMat matrV;
	//double matrU_dat[9*9];
	double matrW_dat[6*4];
	double matrV_dat[4*4];

	//matrU = cvMat(6,6,CV_64FC1,matrU_dat);
	matrW = cvMat(6, 4, CV_64FC1, matrW_dat);
	matrV = cvMat(4, 4, CV_64FC1, matrV_dat);

	CvMat* projPoints[2];
	CvMat* projMatrs[2];

	projPoints[0] = projPoints1;
	projPoints[1] = projPoints2;

	projMatrs[0] = projMatr1;
	projMatrs[1] = projMatr2;

	/* Solve system for each point */
	/* For each point */
	for(int i=0 ; i < numPoints ; ++i) {
		/* Fill matrix for current point */
		/* For each view */
		for(int j=0 ; j < 2 ; ++j) {
			double x,y;
			x = cvmGet(projPoints[j], 0, i);
			y = cvmGet(projPoints[j], 1, i);

			for(int k = 0; k < 4 ; k++) {
				cvmSet(&matrA, j*3+0, k, x * cvmGet(projMatrs[j], 2, k) - cvmGet(projMatrs[j], 0, k));
				cvmSet(&matrA, j*3+1, k, y * cvmGet(projMatrs[j], 2, k) - cvmGet(projMatrs[j], 1, k));
				cvmSet(&matrA, j*3+2, k, x * cvmGet(projMatrs[j], 1, k) - y * cvmGet(projMatrs[j], 0, k));
			}
		}

		/* Solve system for current point */
		{
			cvSVD(&matrA, &matrW, 0, &matrV, CV_SVD_V_T);

			/* Copy computed point */
			cvmSet(points4D, 0, i, cvmGet(&matrV, 3, 0)); /* X */
			cvmSet(points4D, 1, i, cvmGet(&matrV, 3, 1)); /* Y */
			cvmSet(points4D, 2, i, cvmGet(&matrV, 3, 2)); /* Z */
			cvmSet(points4D, 3, i, cvmGet(&matrV, 3, 3)); /* W */
		}
	}

	// save world points
    stringstream ss("");
    ss << RECONS_DIR << "/" << boost::uuids::random_generator()() << ".xml";
    cvSave(ss.str().c_str(), points4D);
	/* Points was reconstructed. Try to re-project points */
	/* We can compute re-projection error if need */
#if REPROJ_ERR
	double err = 0;	
	CvMat point2D;
	CvMat point3D;
	double point3D_dat[4];
	double point2D_dat[3];
	double W;
	double x,y;
	double xr,yr,wr;
	double deltaX,deltaY;

	// allocate memory for matrices
	point3D = cvMat(4, 1, CV_64FC1, point3D_dat);
	point2D = cvMat(3, 1, CV_64FC1, point2D_dat);

	for(int i = 0; i < numPoints; ++i) {
		W = cvmGet(points4D, 3, i);

		point3D_dat[0] = cvmGet(points4D, 0, i) / W;
		point3D_dat[1] = cvmGet(points4D, 1, i) / W;
		point3D_dat[2] = cvmGet(points4D, 2, i) / W;
		point3D_dat[3] = 1.0;

		/* !!! Project this point for each camera */
		for(int currCamera = 0 ; currCamera < 2 ; ++currCamera) {
			cvMatMul(projMatrs[currCamera], &point3D, &point2D);

			x = (double)cvmGet(projPoints[currCamera], 0, i);
			y = (double)cvmGet(projPoints[currCamera], 1, i);

			wr = (double)point2D_dat[2];
			xr = (double)(point2D_dat[0] / wr);
			yr = (double)(point2D_dat[1] / wr);

			deltaX = (double)fabs(x - xr);
			deltaY = (double)fabs(y - yr);

			err += deltaX * deltaX + deltaY * deltaY;
		}
	}
	
	cout << endl << "Re-projection Error : " << sqrt(err / numPoints) << endl;

#endif

	return;
}

