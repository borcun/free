/*
 * Description  : This class consists of all headers, defines, name spaces of RobotScanner Project.
 * Date			: 14.09.2012
 * Author		: Burak Orcun OZKABLAN
 */

#ifndef ABROBOTSCANNERUTIL_H
#define ABROBOTSCANNERUTIL_H

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/legacy/compat.hpp>
#include <osg/ref_ptr>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osgViewer/View>
#include <osgViewer/CompositeViewer>
#include <iostream>
#include <vector>
#include <limits>
#include <cmath>

using namespace std;
using namespace cv;
using namespace osg;
using namespace osgViewer;

// re-projection error macro
// if it is 1, re-projection error is calculated. Otherwise, isn't calculated.
#define REPROJ_ERR 1
#define MAX_CHAR 255
// fundamental elements count
#define FM_ELEM_SIZE 9
// maximum and minimum thresholds when estimating fundamental matrix
#define MAX_THRESHOLD 3.0
#define MIN_THRESHOLD 1.0
// correctness threshold to find fundamental
#define CORRECTNESS_THRESHOLD 0.999999
// threshold values of Levenberg-Marquardt Algorithm
#define LM_THRESHOLD 10e-32
// epipolar distance threshold
#define DIS_THRESHOLD 2.0
// world point coordinate coefficient
#define WPC_COEF 10e+2
// weight and height of camera resolution
#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#define IMG_CHCK_WIDTH 400
#define IMG_CHCK_HEIGHT 310
#define PC_WIN_WIDTH 660
#define PC_WIN_HEIGHT 630
// frame directories
#define LEFT_CAM_DIR "left_camera_images"
#define RIGHT_CAM_DIR "right_camera_images"
#define CORR_DIR "correspondence_images"
#define RECONS_DIR "reconstruction_points"
// frame rate for necessary of reconstruction
#define RECONS_FRAME_RATE 10
// saved point cloud file name
#define PLY_FILE_NAME "surface_point_cloud.ply"
#define PCD_FILE_NAME "surface_point_cloud.pcd"

#endif
