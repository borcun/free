/** 
 * @file:   ABFeatureDetector.hpp
 * @author: dev01
 * @date:   January 21, 2013, 11:56 AM
 */

#ifndef ABFEATUREDETECTOR_HPP
#define	ABFEATUREDETECTOR_HPP

#include "ABRobotScannerUtil.hpp"
#include <boost/thread.hpp>

// define whether to use approximate nearest-neighbor search
#define USE_FLANN

// Feature Detector Types Enumeration
typedef enum Detectors {
    e_SURF = 1, /* Speed Up Robust Features */ 
    e_SIFT,     /* Scale-Invariant Features Transform */
    e_FAST,     /* Features from Accelerated Segment Test */
    e_ORB,      /* oriented BRIEF */
    e_MSER,     /* Maximally Stable Extremal Regions */ 
    e_GFTT,     /* Good Features To Track */ 
    e_DENSE,    /* Dense Feature Detector */ 
    e_SBD       /* Simple Blob Detector */
} DETECTORS;

// Feature Detector Class
class ABFeatureDetector
{
public:
	/// \brief constructor
	ABFeatureDetector();
	/// \brief constructor
	/// @param detector_type - feature detector type
	explicit ABFeatureDetector(const DETECTORS detector_type);
	/// \brief constructor
	/// @param detector_type - feature detector type
	/// @param left_image - left image
	/// @param right_image - right image
	ABFeatureDetector(const DETECTORS detector_type, const IplImage *left_image, const IplImage *right_image);
	/// \brief destructor
	virtual ~ABFeatureDetector();
	/// \brief function that sets detector
	/// @param detector_type - feature detector type
	/// @return -
	void setDetector(const DETECTORS detector_type);
	/// \brief function that sets images
	/// @param left_image - left image
	/// @param right_image - right image
	/// @return -
	void setImages(const IplImage *left_image, const IplImage *right_image);
  	/// \brief function that detects and sign points over image
	/// @param show_result - showing result flag
	/// @return -
	void extractCorrespondedPoints(const bool show_result = false);
    /// \brief functions that get object points
	/// @return std::vector
    std::vector<CvPoint2D64f> getObjectPoints() const;
    /// \brief functions that gets scene points
	/// @return std::vector
    std::vector<CvPoint2D64f> getScenePoints() const;
    
private:
    // feature detector type
    DETECTORS m_detector_type;
    // feature detector
    cv::FeatureDetector *m_detector;
    // descriptor extractor
    cv::DescriptorExtractor *m_extractor;
	// IplImage objects for left and right images
	IplImage *m_object_image_gray;
	IplImage *m_object_image_color;
	IplImage *m_scene_image_gray;
	IplImage *m_scene_image_color;	
	// create an IplImage to show corresponded image
	IplImage *m_correspond_image;
	// create key points and descriptors array
	std::vector<cv::KeyPoint> m_object_keypoints;
	std::vector<cv::KeyPoint> m_scene_keypoints; 
	cv::Mat m_object_descriptors;
	cv::Mat m_scene_descriptors;
    // corresponded points in all images
    std::vector<CvPoint2D64f> m_object_points;
    std::vector<CvPoint2D64f> m_scene_points;
    // corresponded points indexes of all images
    std::vector<int> m_object_indexes;
    std::vector<int> m_scene_indexes;
    // vector that stores matches
    std::vector< std::vector<cv::DMatch> > m_matches;
    // result matrix
    cv::Mat m_results;
    // distance matrix
    cv::Mat m_distances;
    // binary descriptors flag for ORB, BRIEF, BRISK ...
    bool m_is_binarydescriptors;
    // Count of best matches will be found per each query descriptor (or less if it’s not possible).
    int m_best_matches;
    // brute force matcher flag
    bool m_use_BFMatcher;

    /// \brief function that matches the nearest neighbor
	/// @return -
    void matchFlann();
    /// \brief function that processes the nearest neighbor
	/// @return -
    void processFlann();
    
}; // end of class

#endif	/* ABFEATUREDETECTOR_HPP */

