/*
 * Description  : Class that finds fundamental matrix by two point vectors
 * Date			: 10.07.2012
 * Author		: Burak Orcun OZKABLAN
 */

#ifndef FUNDAMENTALMATRIX_H
#define FUNDAMENTALMATRIX_H

#include "ABLevenbergMarquardt.hpp"
#include "ABRobotScannerUtil.hpp"

// Fundamental Matrix Class
class ABFundamentalMatrix
{
public:
	// default constructor
	ABFundamentalMatrix();
	// constructor
	ABFundamentalMatrix(vector<CvPoint2D64f>, vector<CvPoint2D64f>);
	// destructor
	virtual ~ABFundamentalMatrix();
	// function that sets vectors of left and right images points
	void setImagePointVectors(vector<CvPoint2D64f>, vector<CvPoint2D64f>);
	// function that finds fundamental matrix
	CvMat *findFundamentalMat(const double = CORRECTNESS_THRESHOLD);
	// function that gets error of estimation of fundamental matrix
	double getError() const { return m_error; }
	// function that gets correspondence points
	vector<CvPoint2D64f> getLeftImagePoints() const { return m_left_image_points; }
	vector<CvPoint2D64f> getRightImagePoints() const { return m_right_image_points; }

private:
	// fundamental matrix
	CvMat *m_fundamental_matrix;
	// SVD matrices
	CvMat *m_u_mat, *m_d_mat, *m_vt_mat;	
	// left and right image points
	vector<CvPoint2D64f> m_left_image_points;
	vector<CvPoint2D64f> m_right_image_points;
	// error of estimating fundamental matrix
	double m_error;
    
	// function that decomposes fundamental matrix
	void decomposeFundamentalMat();
	// function that refines fundamental matrix with using LevMar algorithm
	void refineFundamentalMatrix();
}; // end of class

#endif