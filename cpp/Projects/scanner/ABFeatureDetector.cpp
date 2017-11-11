#include "ABFeatureDetector.hpp"

// default constructor
ABFeatureDetector::ABFeatureDetector()
{
}

// constructor
ABFeatureDetector::ABFeatureDetector(const DETECTORS detector_type)
{
    // call setDetector function
    setDetector(detector_type);
}

// constructor
ABFeatureDetector::ABFeatureDetector(const DETECTORS detector_type, const IplImage *left_image, const IplImage *right_image)
{
    // set detector type
    m_detector_type = detector_type;
	// set images
	setImages(left_image, right_image);
}

// destructor
ABFeatureDetector::~ABFeatureDetector()
{
	// release images
	cvReleaseImage(&m_object_image_color);
	cvReleaseImage(&m_object_image_gray);
	cvReleaseImage(&m_scene_image_color);	
	cvReleaseImage(&m_scene_image_gray);
    // delete pointer
    delete m_detector;
    delete m_extractor;
    // clear vectors
    m_object_keypoints.clear();
    m_scene_keypoints.clear();
    m_object_points.clear();
    m_scene_points.clear();
    m_object_indexes.clear();
    m_scene_indexes.clear();
    m_matches.clear();
}

// function that sets detector
void ABFeatureDetector::setDetector(const DETECTORS detector_type)
{
    // set detector type
    m_detector_type = detector_type;
}

// function that sets image and if necessary, it converts image attributes
void ABFeatureDetector::setImages(const IplImage *left_image, const IplImage *right_image)
{
	// check depths and channels of parameter images
	// if depths and channels are available for gray image, enter if block
	if(left_image->depth == IPL_DEPTH_8U && left_image->nChannels == 1 && right_image->depth == IPL_DEPTH_8U && right_image->nChannels == 1) 
	{
		// allocate memory for left and right images
		m_object_image_color = cvCreateImage(cvGetSize(left_image),  IPL_DEPTH_8U, 3);
		m_scene_image_color  = cvCreateImage(cvGetSize(right_image), IPL_DEPTH_8U, 3);
		m_object_image_gray  = cvCreateImage(cvGetSize(left_image),  IPL_DEPTH_8U, 1);
		m_scene_image_gray   = cvCreateImage(cvGetSize(right_image), IPL_DEPTH_8U, 1);	

		// convert gray image to color image
		cvCvtColor(left_image, m_object_image_color, CV_GRAY2BGR);
		cvCvtColor(right_image, m_scene_image_color, CV_GRAY2BGR);

		// copy parameters to images
		cvCopy(left_image, m_object_image_gray);
		cvCopy(right_image, m_scene_image_gray);
	}
	// if depths and channels are available for RGB image, enter else if block
	else if(left_image->depth == IPL_DEPTH_8U && left_image->nChannels == 3 && right_image->depth == IPL_DEPTH_8U && right_image->nChannels == 3) 
	{
		// allocate memory for left and right images
		m_object_image_color = cvCreateImage(cvGetSize(left_image),  IPL_DEPTH_8U, 3);
		m_scene_image_color  = cvCreateImage(cvGetSize(right_image), IPL_DEPTH_8U, 3);        
		m_object_image_gray  = cvCreateImage(cvGetSize(left_image),  IPL_DEPTH_8U, 1);
		m_scene_image_gray   = cvCreateImage(cvGetSize(right_image), IPL_DEPTH_8U, 1);	

		// copy rgb image to color image
		cvCopy(left_image, m_object_image_color);
		cvCopy(right_image, m_scene_image_color);
		// convert RGB images to gray images
		cvCvtColor(left_image, m_object_image_gray, CV_BGR2GRAY);
		cvCvtColor(right_image, m_scene_image_gray, CV_BGR2GRAY);
	}
	// if images are not available to process, give an error message
	else {
		cerr << "Images must be GRAY (IPL_DEPTH_8U and 1 channel) or RGB (IPL_DEPTH_8U and 3 channels)" << endl;
		m_correspond_image = m_object_image_color = m_object_image_gray = m_scene_image_color = m_scene_image_gray = NULL;
	}
}

// function that detects and sign points over image by show_result parameter
void ABFeatureDetector::extractCorrespondedPoints(const bool show_result)
{    
	// initialize non-free module
	cv::initModule_nonfree();
    
    // According to detector type, select a special process
    switch(m_detector_type) {
        case e_SURF:
            // create a SURF parameter that contains Hessian threshold
            // int extended; // 0 means basic descriptors (64 elements each) 1 means extended descriptors (128 elements each)
            // double hessian_threshold; // only features with key point.Hessian larger than that are extracted.
            // good default value is ~300-500 (can depend on the average local contrast and sharpness of the image).
            // user can further filter out some features based on their Hessian values and other characteristics.
            m_detector  = new cv::SURF(300, 1);
            m_extractor = new cv::SURF(300, 1);
             // set to false to use Brute Force Matcher
            m_use_BFMatcher = false;
            m_is_binarydescriptors = false;
        break;
        // Scale-Invariant Feature Transform
        case e_SIFT:
            m_detector  = new cv::SIFT();
            m_extractor = new cv::SIFT();        
             // set to false to use Brute Force Matcher
            m_use_BFMatcher = false;
            m_is_binarydescriptors = false;            
        break;
        // Features from Accelerated Segment Test
        case e_FAST:
            m_detector  = new cv::FastFeatureDetector();
            m_extractor = new cv::FREAK();
             // set to false to use Brute Force Matcher
            m_use_BFMatcher = false;
            m_is_binarydescriptors = false;
        break;
        // Oriented BRIEF
        case e_ORB:
            m_detector  = new cv::ORB();
            m_extractor = new cv::ORB();        
             // set to true to use Brute Force Matcher (may give better results with binary descriptors)
            m_use_BFMatcher = true;     
            m_is_binarydescriptors = true;
        break;        
        // Maximally Stable Extremal Regions
        case e_MSER:
            m_detector  = new cv::MSER();
            m_extractor = new cv::FREAK();        
             // set to false to use Brute Force Matcher
            m_use_BFMatcher = false;     
            m_is_binarydescriptors = false;
        break;
        // Good Features To Track
        case e_GFTT:
            m_detector  = new cv::GFTTDetector();
            m_extractor = new cv::FREAK();
             // set to false to use Brute Force Matcher
            m_use_BFMatcher = false;       
            m_is_binarydescriptors = false;
        break;
        // Dense Feature Detector
        case e_DENSE:
            m_detector  = new cv::DenseFeatureDetector();
            m_extractor = new cv::FREAK();
             // set to false to use Brute Force Matcher
            m_use_BFMatcher = false;      
            m_is_binarydescriptors = false;
        break;
        // Simple Blob Detector
        case e_SBD:
            m_detector  = new cv::SimpleBlobDetector();
            m_extractor = new cv::FREAK();     
             // set to false to use Brute Force Matcher
            m_use_BFMatcher = false;            
            m_is_binarydescriptors = false;
        break;
        // Invalid Detector
        default:
            cerr << "Please, type valid Feature Detector Type" << endl;
        return;
    } // end of switch
    
    // detect key points of images
    m_detector->detect(m_object_image_gray, m_object_keypoints);
    m_detector->detect(m_scene_image_gray, m_scene_keypoints);
    // compute descriptions of images
    m_extractor->compute(m_object_image_gray, m_object_keypoints, m_object_descriptors);
    m_extractor->compute(m_scene_image_gray, m_scene_keypoints, m_scene_descriptors);

    // match FLANN
    matchFlann();
    // process FLANN
    processFlann();
    
    if(show_result) {
        // print information about how many descriptors and key points are found
        cout << "Object Key Points  : " << (int)m_object_keypoints.size() << endl;
        cout << "Scene Key Points   : " << (int)m_scene_keypoints.size() << endl;
        cout << "Object Descriptors : " << m_object_descriptors.rows << endl;
        cout << "Scene Descriptors  : " << m_scene_descriptors.rows  << endl;

        // allocate memory for correspondedImage
        m_correspond_image = cvCreateImage(cvSize(m_object_image_color->width * 2, m_object_image_color->height), IPL_DEPTH_8U, 3);	
        // insert object and scene images into corresponded image
        cvSetImageROI(m_correspond_image, cvRect(0, 0, m_object_image_color->width, m_object_image_color->height));
        // copy object images corresponded images
        cvCopy(m_object_image_color, m_correspond_image);
        // set new roi for scene images
        cvSetImageROI(m_correspond_image, cvRect(m_object_image_color->width, 0, m_correspond_image->width, m_correspond_image->height));
        // copy scene images to corresponded images
        cvCopy(m_scene_image_color, m_correspond_image);
        // reset corresponded images
        cvResetImageROI(m_correspond_image);

        // create point variables of object and scene images
        cv::KeyPoint kp_object, kp_scene;
        CvPoint center_object, center_scene;
        int radius_object, radius_scene;

        // Because of m_object_points.size equals m_scene_points.size, choose one of them to process in loop
        // Draw circles for key points, draw lines between corresponded key points (circles)
        for(int i = 0; i < (int)m_object_points.size() ; ++i) {
            // get object image points
            kp_object.pt.x = m_object_points.at(i).x;
            kp_object.pt.y = m_object_points.at(i).y;
            // get scene image points
            kp_scene.pt.x = m_scene_points.at(i).x;
            kp_scene.pt.y = m_scene_points.at(i).y;
            // get center of points in object image
            center_object.x = cvRound(kp_object.pt.x);
            center_object.y = cvRound(kp_object.pt.y);
            radius_object   = cvRound(kp_object.size * 1.2 / 9. * 2);
            // get center of points in scene image
            center_scene.x = cvRound(kp_scene.pt.x + m_object_image_color->width);
            center_scene.y = cvRound(kp_scene.pt.y);
            radius_scene   = cvRound(kp_scene.size * 1.2 / 9. * 2);
            // draw two circle on correlative correspondence image from one of object image, one of scene image
            cvCircle(m_correspond_image, center_object, radius_object, cvScalar(150, 0, 0), 2, 8, 0);
            cvCircle(m_correspond_image, center_scene, radius_scene, cvScalar(0, 0, 150), 2, 8, 0);
            // draw a line between circles
            cvLine(m_correspond_image, center_object, center_scene, cvScalar(0, 150, 0), 1, 8);
        } // end of for

        // show correspondedImageColor
        cout << "Correlative Corresponded Points : " << m_object_points.size() << endl;
        cvNamedWindow("Correlative_Correspondence", CV_WINDOW_AUTOSIZE);
        cvShowImage("Correlative_Correspondence", m_correspond_image);
        cvWaitKey(10000);
        cvDestroyWindow("Correlative_Correspondence");
    } // end of if(show_result)
    
	return;
}

// function that find the nearest neighbor
void ABFeatureDetector::matchFlann()
{
    // find the 2 nearest neighbors
    m_best_matches = 2;

    // if object descriptors type is unsigned integer, it is binary descriptor
    // Binary descriptors detected (from ORB, Brief, BRISK, FREAK)
    if(m_object_descriptors.type() == CV_8U) { 
        // check Brute Force Matcher, if it is true, make comparison Brute Force
        if(m_use_BFMatcher) {
            // create a Brute Force Matcher object
            cv::BFMatcher matcher(cv::NORM_HAMMING);
            // search by brute force (nearest neighbor)
            matcher.knnMatch(m_object_descriptors, m_scene_descriptors, m_matches, m_best_matches);
        }
        else {            
            // LsHIndexParams(f, s, t);
            // When using a parameters object of this type the index created uses multi-probe LSH 
            // first parameter is table_number the number of hash tables to use (between 10 and 30 usually).
            // second parameter is key_size the size of the hash key in bits (between 10 and 20 usually).
            // third parameter is multi_probe_level the number of bits to shift to check for neighboring buckets (0 is regular LSH, 2 is recommended).
            
            // Create FLANN LSH index
            cv::flann::Index flannIndex(m_scene_descriptors, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_MINKOWSKI /*FLANN_DIST_HAMMING*/);
            // Results index
            m_results = cv::Mat(m_object_descriptors.rows, m_best_matches, CV_32SC1);
            // Distance results are CV_32FC1
            m_distances = cv::Mat(m_object_descriptors.rows, m_best_matches, CV_32FC1);
            // search (nearest neighbor)
            flannIndex.knnSearch(m_object_descriptors, m_results, m_distances, m_best_matches, cv::flann::SearchParams() );
        }
    } // end of if
    else {        
        if(m_use_BFMatcher) {
            // create a Brute Force Matcher object
            cv::BFMatcher matcher(cv::NORM_L2);
            // search by brute force (nearest neighbor)
            matcher.knnMatch(m_object_descriptors, m_scene_descriptors, m_matches, m_best_matches);
        }
        else {
            // KDTreeIndexParams(f, s, t, fo)
            // first parameter is branching The branching factor to use for the hierarchical k-means tree
            // second parameter is iterations The maximum number of iterations to use in the k - means clustering stage when building the k - means tree. 
            // A value of - 1 used here means that the k - means clustering should be iterated until convergence
            // Third parameter is centers_init The algorithm to use for selecting the initial centers when performing a k - means clustering step. 
            // The possible values are CENTERS_RANDOM(picks the initial cluster centers randomly), CENTERS_GONZALES(picks the initial centers using Gonzales’ algorithm) and 
            // CENTERS_KMEANSPP(picks the initial centers using the algorithm suggested in arthur_kmeanspp_2007)
            // Fourth  parameter is cb_index This parameter(cluster boundary index) influences the way exploration is performed in the hierarchical k-means tree. 
            // When cb_index is zero the next k-means domain to be explored is chosen to be the one with the closest center. 
            // A value greater then zero also takes into account the size of the domain.
            
            // Create FLANN KDTree index
            cv::flann::Index flannIndex(m_scene_descriptors, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
            // Results index
            m_results = cv::Mat(m_object_descriptors.rows, m_best_matches, CV_32SC1);
            // Distance results are CV_32FC1
            m_distances = cv::Mat(m_object_descriptors.rows, m_best_matches, CV_32FC1);
            // search (nearest neighbor)
            flannIndex.knnSearch(m_object_descriptors, m_results, m_distances, m_best_matches, cv::flann::SearchParams());
        }
    } // end of else
    
    return;
}

// function that process FLANN
void ABFeatureDetector::processFlann()
{
    // first nndrRatio is 0.6 but results are not good
    // so it was changed and made as 0.3
    float nndrRatio = 0.3;
    CvPoint2D64f point;
    
    // Check if this descriptor matches with those of the objects
    if(!m_use_BFMatcher) {
        // Apply Nearest Neighbor Distance Ratio
        // Binary, just take the nearest
        for(int i=0 ; i < m_object_descriptors.rows ; ++i) {
            if(m_is_binarydescriptors || m_distances.at<float>(i,0) <= nndrRatio * m_distances.at<float>(i,1)) {
                // push object points and its index
                point.x = m_object_keypoints.at(i).pt.x;
                point.y = m_object_keypoints.at(i).pt.y;                
                m_object_points.push_back(point);
                m_object_indexes.push_back(i);
                // push scene points and its index
                point.x = m_scene_keypoints.at(m_results.at<int>(i,0)).pt.x;
                point.y = m_scene_keypoints.at(m_results.at<int>(i,0)).pt.y;                
                m_scene_points.push_back(point);
                m_scene_indexes.push_back(m_results.at<int>(i,0));
            }
        }
    } // end of if
    else {
        // Apply Nearest Neighbor Distance Ratio
        for(int i=0 ; i < (int)m_matches.size() ; ++i) {
            if(m_is_binarydescriptors || m_matches.at(i).at(0).distance <= nndrRatio * m_matches.at(i).at(1).distance) {
                // push object points and its index
                point.x = m_object_keypoints.at(m_matches.at(i).at(0).queryIdx).pt.x;
                point.y = m_object_keypoints.at(m_matches.at(i).at(0).queryIdx).pt.y;
                m_object_points.push_back(point);
                m_object_indexes.push_back(m_matches.at(i).at(0).queryIdx);
                // push scene points and its index
                point.x = m_scene_keypoints.at(m_matches.at(i).at(0).trainIdx).pt.x;
                point.y = m_scene_keypoints.at(m_matches.at(i).at(0).trainIdx).pt.y;
                m_scene_points.push_back(point);
                m_scene_indexes.push_back(m_matches.at(i).at(0).trainIdx);
            }
        }
    } // end of else

    return;
}

// function that gets object points
std::vector<CvPoint2D64f> ABFeatureDetector::getObjectPoints() const 
{ 
	return m_object_points; 
}

// function that gets scene points
std::vector<CvPoint2D64f> ABFeatureDetector::getScenePoints() const 
{ 
	return m_scene_points; 
}
