/**
 * @file:   ABReconstruction.hpp
 * @title:  This class calculates projection matrices, triangulates correspondence points and finds world points
 * @author: Burak Orcun OZKABLAN
 * @date:   14.09.2012
 */

#ifndef ABRECONSTRUCTION_H
#define ABRECONSTRUCTION_H

#include "ABRobotScannerUtil.hpp"
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

// ABReconstruction Class
class ABReconstruction
{
public:
	/// \brief default constructor
	ABReconstruction();
	/// \brief constructor
	/// @param intrinsic_matrix_1 - left camera intrinsic matrix
	/// @param intrinsic_matrix_2 - right camera intrinsic matrix
	/// @param rotation_matrix - rotation matrix
	/// @param translation_matrix - translation matrix
	ABReconstruction(const CvMat *intrinsic_matrix_1, const CvMat *intrinsic_matrix_2, const CvMat *rotation_matrix, const CvMat *translation_matrix);
	/// \brief destructor
	virtual ~ABReconstruction();
	/// \brief function that sets intrinsic matrices
	/// @param intrinsic_matrix_1 - left camera intrinsic matrix
	/// @param intrinsic_matrix_2 - right camera intrinsic matrix
	/// @return -
	void setIntrinsicMatrices(const CvMat *intrinsic_matrix1, const CvMat *intrinsic_matrix2);
	/// \brief function that sets rotation matrix
	/// @param rotation_matrix - rotation matrix
	/// @return -
	void setRotationMatrix(const CvMat *rotation_matrix);
	/// \brief function that sets translation matrix
	/// @param translation_matrix - translation matrix
	/// @return -
	void setTranslationMatrix(const CvMat *translation_matrix);
	/// \brief function that set first projection matrix
	/// @return CvMat pointer
	CvMat *getFirstProjectionMatrix();
	/// \brief function that set second projection matrix
	/// @return CvMat pointer
	CvMat *getSecondProjectionMatrix();
	/// \brief function that makes back-projection and extract 3D Model
	/// @param left_points - left image points
	/// @param right_points - right image points
	/// @param fundamental matrix - fundamental matrix
	/// @return -
	void triangulatePoints(vector<CvPoint2D64f> left_points, vector<CvPoint2D64f> right_points, CvMat *fundamental_matrix);
	/// \brief function that gets world coordinate points
	/// @return CvMat pointer
	CvMat *getWorldPoints() const;

private:
	// intrinsic matrices
	CvMat *m_intrinsic_matrix1;
	CvMat *m_intrinsic_matrix2;
	// rotation matrix
	CvMat *m_rotation_matrix;
	// translation matrix
	CvMat *m_translation_matrix;
	// projection matrices
	CvMat *m_projection_matrix1;
	CvMat *m_projection_matrix2;
	// corresponded image points and world points
	CvMat *m_left_image_points;
	CvMat *m_right_image_points;
	CvMat *m_left_image_points1D;
	CvMat *m_right_image_points1D;
	CvMat *m_correct_left_image_points;
	CvMat *m_correct_right_image_points;
	CvMat *m_world_points;

	/// \brief function that triangulates points
	/// @param projection_matrix_1 - left camera projection matrix
	/// @param projection_matrix_2 - right camera projection matrix
	/// @param projected_points_1 - projected points on left camera
	/// @param projected_points_2 - projected points on right camera
	/// @param points4D - world coordinate points
	/// @return -
	void triangulate(CvMat* projection_matrix_1, CvMat* projection_matrix_2, CvMat* projected_points_1, CvMat* projected_points_2, CvMat* points4D);

}; // end of class

#endif
