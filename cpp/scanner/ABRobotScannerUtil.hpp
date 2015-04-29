/**
 * @file:   ABRobotScannerUtil.hpp
 * @title:  ABRobotScannerUtil Header
 * @author: Burak Orcun OZKABLAN
 * @date:   01.04.2013
 */

#ifndef ABROBOTSCANNERUTIL_HPP
#define ABROBOTSCANNERUTIL_HPP

// Qt
#include <QtGui/QApplication>
#include <QImage>
#include <QMessageBox>
// OpenCV
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
// boost
#include <boost/thread.hpp>
#include <boost/smart_ptr/make_shared.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
// ximea
#include <ximea/xiApi.h>
#include <ximea/xiExt.h>
#include <ximea/m3Api.h>
#include <ximea/m3Ext.h>
// standart libs
#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <string>
#include <sstream>
#include <cstdlib>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
// windows
#include <Windows.h>

using namespace std;
using namespace cv;

#define DISTORTION_AND_NOISE 0
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
#define WPC_COEF 1.0
// weight and height of camera resolution
#define IMG_WIDTH		  2040
#define IMG_HEIGHT		  2040
#define PC_WIDTH		  950
#define PC_HEIGHT		  480
#define STREAM_WIN_WIDTH  480
#define STREAM_WIN_HEIGHT 480
// frame directories
#define LEFT_CAM_DIR  "left_camera_images"
#define RIGHT_CAM_DIR "right_camera_images"
#define RECONS_DIR	  "reconstruction_points"
#define CORR_DIR	  "correspondence_images"
// frame rate for necessary of reconstruction
#define RECONS_FRAME_RATE 1
// saved point cloud file name
#define XYZ_FILE_NAME "surface_point_cloud.xyz"
#define PLY_FILE_NAME "surface_point_cloud.ply"

#endif
