/**
 * @title: This class draws epipolar lines.
 * @file: ABEpipolarLine.hpp 
 * @author: Burak Orcun OZKABLAN
 * @date: 14.09.2012
 */

#ifndef EPIPOLARLINE
#define EPIPOLARLINE

#include "ABRobotScannerUtil.hpp"

// Epipolar Line Class
class ABEpipolarLine
{
public:
	/// \brief default constructor
	ABEpipolarLine();
	/// \brief constructor
	/// @param feature_points - image feature points
	/// @param fundamental_matrix - fundamental matrix
	ABEpipolarLine(const vector<CvPoint2D64f> feature_points, const CvMat *fundamental_matrix);
	/// \brief destructor
	virtual ~ABEpipolarLine();
	/// \brief function that sets image points
	/// @param feature_points - image feature points
	/// @return -
	void setPoints(const vector<CvPoint2D64f> feature_points);
	/// \brief function that sets fundamental matrix
	/// @param fundamental_matrix - fundamental matrix
	/// @return -
	void setFundamentalMatrix(const CvMat *fundamental_matrix);
	/// \brief function that runs finding epipolar line algorithm and stores all found epipolar line equations
    /// @param which_image - it shows image number
	/// @return -
	void findEpipolarLines(const int which_image);
	/// \brief function that finds epipolar points 
	/// @param image_width - width of one image from both of them
	void findEpipolarPoints(const int image_width);
	/// \brief function that gets epipolar lines
	/// @return CvMat pointer
	CvMat *getEpipolarLines() const;
	/// \brief function that gets start and finish points of epipolar lines
	/// @return CvMat pointer
	CvMat *getEpipolarPoints() const;
	/// \brief functions that draw epipolar lines
	/// @param image - IplImage whose surface is drawn
	/// @param object_points - object points from feature detector
	/// @param scene_points - scene points from feature detector
	/// @param epi_lines - epipolar lines
	/// @param epi_points - epipolar points
	/// @return -
	void drawEpipolarLines(IplImage *image, vector<CvPoint2D64f> object_points, vector<CvPoint2D64f> scene_points, CvMat *epi_lines, CvMat *epi_points);

private:
	CvMat *m_feature_points;
	CvMat *m_fundamental_matrix;
	CvMat *m_epipolar_lines;
	CvMat *m_epipolar_points;

	/// \brief function that finds distance of point to a vector and decided if distance is big or not line equation is \b ax+by+c=0
	/// @param a - a
	/// @param b - b
	/// @param c - c
	/// @param x - x
	/// @param y - y
	/// @param threshold - threshold value 
	inline bool euclideanDistance(const double a, const double b, const double c, const double x, const double y, const double threshold = DIS_THRESHOLD);

};

#endif
