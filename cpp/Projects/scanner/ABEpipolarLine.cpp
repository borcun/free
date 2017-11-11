#include "ABEpipolarLine.hpp"

// default constructor
ABEpipolarLine::ABEpipolarLine()
{

}

// constructor
ABEpipolarLine::ABEpipolarLine(const vector<CvPoint2D64f> feature_points, const CvMat *fundamental_matrix)
{
	setPoints(feature_points);
	setFundamentalMatrix(fundamental_matrix);
}

// destructor
ABEpipolarLine::~ABEpipolarLine()
{
	// release matrices
	cvReleaseMat(&m_feature_points);
	cvReleaseMat(&m_fundamental_matrix);
	cvReleaseMat(&m_epipolar_lines);
	cvReleaseMat(&m_epipolar_points);
}

// function that sets image points
void ABEpipolarLine::setPoints(const vector<CvPoint2D64f> feature_points)
{
	m_feature_points = cvCreateMat(2, (int)feature_points.size(), CV_64FC1);
	// copy vector to CvMat
	for(int i=0 ; i < m_feature_points->cols ; ++i) {
		CV_MAT_ELEM(*m_feature_points, double, 0, i) = feature_points.at(i).x;
		CV_MAT_ELEM(*m_feature_points, double, 1, i) = feature_points.at(i).y;
	}
}

// function that sets fundamental matrix
void ABEpipolarLine::setFundamentalMatrix(const CvMat *fundamental_matrix)
{
	m_fundamental_matrix = cvCreateMat(fundamental_matrix->rows, fundamental_matrix->cols, fundamental_matrix->type);
	cvCopy(fundamental_matrix, m_fundamental_matrix);
}

// function that runs finding epipolar line algorithm and stores all found epipolar line equations
void ABEpipolarLine::findEpipolarLines(const int which_image)
{
	m_epipolar_lines = cvCreateMat(3, m_feature_points->cols > m_feature_points->rows ? m_feature_points->cols : m_feature_points->rows, m_feature_points->type);
	// find epipolar lines
	cvComputeCorrespondEpilines(m_feature_points, which_image, m_fundamental_matrix, m_epipolar_lines);
}

// function that draws epipolar lines 
void ABEpipolarLine::findEpipolarPoints(const int image_width)
{
	m_epipolar_points = cvCreateMat(4, m_epipolar_lines->cols, CV_64FC1);
	register double y1, y2;

	for(int i=0 ; i < m_epipolar_lines->cols ; ++i) {
		// find x1,y1 points
		y1 = (-CV_MAT_ELEM(*m_epipolar_lines, double, 2, i) - (CV_MAT_ELEM(*m_epipolar_lines, double, 0, i) * 0)) / CV_MAT_ELEM(*m_epipolar_lines, double, 1, i);
		// find x2,y2 points
		y2 = (-CV_MAT_ELEM(*m_epipolar_lines, double, 2, i) - (CV_MAT_ELEM(*m_epipolar_lines, double, 0, i) * image_width)) / CV_MAT_ELEM(*m_epipolar_lines, double, 1, i);
		
		CV_MAT_ELEM(*m_epipolar_points, double, 0, i) = 0.0;
		CV_MAT_ELEM(*m_epipolar_points, double, 1, i) = y1;
		CV_MAT_ELEM(*m_epipolar_points, double, 2, i) = (double)image_width;
		CV_MAT_ELEM(*m_epipolar_points, double, 3, i) = y2;
	}

	return;
}

// function that find corresponded epipolar lines of objectPoints
void ABEpipolarLine::drawEpipolarLines(IplImage *image, vector<CvPoint2D64f> object_points, vector<CvPoint2D64f> scene_points, CvMat *epi_lines, CvMat *epi_points)
{
	for(int i=0 ; i < (int)object_points.size() ; ++i) {
		if(euclideanDistance(CV_MAT_ELEM(*epi_lines, double, 0, i), 
							 CV_MAT_ELEM(*epi_lines, double, 1, i), 
							 CV_MAT_ELEM(*epi_lines, double, 2, i), scene_points.at(i).x, scene_points.at(i).y)) 
		{
			// draw epipolar lines
			cvLine( image, 
					cvPoint((int)CV_MAT_ELEM(*epi_points, double, 0, i), (int)CV_MAT_ELEM(*epi_points, double, 1, i)),
					cvPoint((int)CV_MAT_ELEM(*epi_points, double, 2, i), (int)CV_MAT_ELEM(*epi_points, double, 3, i)),
					cvScalar(0, 255, 0, 0),
					1, // thickness
					4  // Bresenham line connected number
			);

            cvCircle(image, cvPoint((int)scene_points.at(i).x, (int)scene_points.at(i).y), 1, cvScalar(255,0,0,0), 2);
		}
		//else {
		//	// draw epipolar lines
		//	cvLine( image, 
		//		cvPoint(CV_MAT_ELEM(*epiPoints, double, 0, i), CV_MAT_ELEM(*epiPoints, double, 1, i)),
		//		cvPoint(CV_MAT_ELEM(*epiPoints, double, 2, i), CV_MAT_ELEM(*epiPoints, double, 3, i)),
		//		cvScalar(0,0,255,0),
		//		1, // thickness
		//		4  // Bresenham line connected number
		//		);
		//}
		//// draw corresponded point
		//cvCircle(image, cvPoint(scenePoints.at(i).x, scenePoints.at(i).y), 1, cvScalar(0,255,255,0), 2);
	}
}

// function that gets epipolar lines
CvMat *ABEpipolarLine::getEpipolarLines() const 
{ 
	return m_epipolar_lines; 
}

// function that gets start and finish points of epipolar lines
CvMat *ABEpipolarLine::getEpipolarPoints() const 
{ 
	return m_epipolar_points; 
}

/// function that finds distance of point to a vector and decided if distance is big or not line equation is ax + by + c = 0
bool ABEpipolarLine::euclideanDistance(const double a, const double b, const double c, const double x, const double y, const double threshold) 
{
	return fabs(a * x + b * y + c) < threshold;
}
