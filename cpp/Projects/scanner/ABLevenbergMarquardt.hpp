/*
 * Description  : This class consists of Levenberg Marquardt algorithm to optimize fundamental matrix.
 * Date			: 14.09.2012
 * Author		: Burak Orcun OZKABLAN
 */

#ifndef LEVENBERGMARQUARDT_H
#define LEVENBERGMARQUARDT_H

#include "ABRobotScannerUtil.hpp"

// Levenberg Marquardt Class
class ABLevenbergMarquardt
{
public:
	/// \brief default constructor
	ABLevenbergMarquardt();
	/// \brief destructor
	virtual ~ABLevenbergMarquardt();
	/// \brief function that run LM algorithm and optimize matrix
	/// @param jacobian_matrix - Jacobian matrix
	/// @param error_matrix - error matrix
	/// @param fundamental_matrix - fundamental matrix
	/// @param left_points - left image points
	/// @param right_points - right image points
	/// @param criteria - criteria value for precision of fundamental matrix
	/// @return CvMat pointer
	CvMat *run(CvMat *jacobian_matrix, CvMat *error_matrix, CvMat *fundamental_matrix, const vector<CvPoint2D64f> left_points, const vector<CvPoint2D64f> right_points,
													const CvTermCriteria = cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, DBL_EPSILON));
	/// \brief function that creates Jacobian matrix
	/// @param left_points - left image points
	/// @param right_points - right image points
	/// @param fundamental_matrix - fundamental matrix
	/// @return -
	void createJacobianMatrix(const vector<CvPoint2D64f> left_points, const vector<CvPoint2D64f> right_points, const CvMat *fundamental_matrix);
	/// \brief function that gets Jacobian matrix
	/// @return CvMat pointer
	CvMat *getJacobianMatrix() const;
	/// \brief function that gets error matrix
	/// @return CvMat pointer
	CvMat *getErrorMatrix() const;

private:
	// Jacobian matrix
	CvMat *JMatFM;
	// error matrix
	CvMat *errMatFM;
	// others matrices
	CvMat *JtMat;
	CvMat *JtJMat;
	CvMat *epsMat;
	CvMat *JtEpsMat;
	// fundamental matrix variables
	CvMat *currentMatF;
	// delta, its transpose and lambda . delta matrices
	CvMat *deltaMat;
	CvMat *deltaMatT;
	CvMat *lambdaDeltaMat;
	double lambda;
	double ro;
	double lambdaIncCoef;

	/// \brief function that gets new epsilon matrix for fundamental matrix
	/// @param left_points - left image points
	/// @param right_points - right image points
	/// @param fundamental_matrix - fundamental matrix
	/// @return CvMat pointer
	CvMat *getEpsMatrix(const vector<CvPoint2D64f> left_points, const vector<CvPoint2D64f> right_points, const CvMat *fundamental_matrix);
	/// \brief function that gets infinity norm
	/// @param matrix - matrix
	/// @return double
	inline double getInfinityNorm(const CvMat *matrix);
	/// \brief function that gets maximum diagonal element
	/// @param matrix - matrix
	/// @return double
	inline double getMaxDiagonalElement(const CvMat *matrix);
	/// \brief function that converts 9x1 line matrix to 3x3 square matrix
	/// @param row_matrix - row matrix
	/// @return CvMat pointer
	inline CvMat *convert9x1to3x3(const CvMat *row_matrix);

}; // end of class

#endif