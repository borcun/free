/*
 * @file:   ABEssentialMatrix.hpp
 * @title:  Class that finds essential matrix by intrinsic matrix and fundamental matrix
 * @author: Burak Orcun OZKABLAN
 * @date:   26.07.2012
 */

#ifndef ESSENTIALMATRIX_H
#define ESSENTIALMATRIX_H

#include "ABRobotScannerUtil.hpp"
#include "ABLevenbergMarquardt.hpp"

// Essential Matrix Class
class ABEssentialMatrix
{
public:
	/// \brief default constructor
	ABEssentialMatrix();
	/// \brief constructor
	/// @param intrinsic_matrix_1 - left camera intrinsic matrix
	/// @param intrinsic_matrix_2 - right camera intrinsic matrix
	/// @param fundamental_matrix - fundamental matrix
	ABEssentialMatrix(const CvMat *intrinsic_matrix_1, const CvMat *intrinsic_matrix_2, const CvMat *fundamental_matrix);
	/// \brief constructor
	/// @param left_points - left image points
	/// @param right_points - right image points
	/// @param intrinsic_matrix_1 - left camera intrinsic matrix
	/// @param intrinsic_matrix_2 - right camera intrinsic matrix
	/// @param fundamental_matrix - fundamental matrix
	ABEssentialMatrix(vector<CvPoint2D64f> left_points, vector<CvPoint2D64f> right_points, const CvMat *intrinsic_matrix_1, const CvMat *intrinsic_matrix_2, const CvMat *fundamental_matrix);
	/// \brief destructor
	virtual ~ABEssentialMatrix();
	/// \brief function that sets intrinsic matrices
	/// @param intrinsic_matrix_1 - left camera intrinsic matrix
	/// @param intrinsic_matrix_2 - right camera intrinsic matrix
	/// @return -
	void setIntrinsicMatrices(const CvMat *intrinsic_matrix_1, const CvMat *intrinsic_matrix_2);
	/// \brief function that sets fundamental matrix
	/// @param fundamental_matrix - fundamental matrix
	/// @return -
	void setFundamentalMatrix(const CvMat *fundamental_matrix);
 	/// \brief function that sets vectors of left and right images points
	/// @param left_points - left image points
	/// @param right_points - right image points
	/// @return -
	void setImagePointVectors(vector<CvPoint2D64f> left_points, vector<CvPoint2D64f> right_points);
	/// \brief function that finds essential matrix
	/// @return CvMat pointer
	CvMat *findEssentialMat();
	/// \brief functions that gets first rotation matrix
	/// @return CvMat pointer
	CvMat *getRotationMatrix1() const;
	/// \brief functions that gets second rotation matrix
	/// @return CvMat pointer
	CvMat *getRotationMatrix2() const;
	/// \brief functions that gets first translation matrix
	/// @return CvMat pointer
	CvMat *getTranslationMatrix1() const;
	/// \brief functions that gets second translation matrix
	/// @return CvMat pointer
	CvMat *getTranslationMatrix2() const;
	/// \brief function that gets error of SVD process
	/// @return double
	double getError() const;

private:
	// essential matrix
	CvMat *m_essential_matrix;
	// fundamental matrix
	CvMat *m_fundamental_matrix;
	// rotation and translation matrices
	CvMat *m_rotation_matrix_1;
	CvMat *m_rotation_matrix_2;
	CvMat *m_translation_matrix_1;
	CvMat *m_translation_matrix_2;
	// intrinsic matrices
	CvMat *m_intrinsic_matrix_1;
	CvMat *m_intrinsic_matrix_2;
	// intrinsic matrices inverses
	CvMat *m_intrinsic_matrix_inv_1;
	CvMat *m_intrinsic_matrix_inv_2;
  	// left and right image points
	vector<CvPoint2D64f> m_left_image_points;
	vector<CvPoint2D64f> m_right_image_points;
	// SVD matrices
	CvMat *m_u_mat, *m_d_mat, *m_vt_mat;
	// error of SVD
	double m_error;
    
	/// \brief function that decomposes essential matrix and fill rotation and translation matrices. 
	/// \brief The formulas are from Multiple View Geometry [ A. Zisserman and R. Hartley ]
	/// @return -
	void decomposeEssentialMat();
	/// \brief function that refines essential matrix with using LevMar algorithm
	/// @return -
	void refineEssentialMatrix();

}; // end of ABEssentialMatrix class

#endif