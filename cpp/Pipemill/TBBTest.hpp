/* 
 * File:   TBBTest.hpp
 * Author: dev01
 *
 * Created on January 25, 2013, 9:37 AM
 */

#ifndef TBBTEST_HPP
#define	TBBTEST_HPP

#include <tbb/task_scheduler_init.h>
#include <tbb/pipeline.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <iostream>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;
using namespace tbb;

struct timeval start_time, end_time;
long mtime;
long seconds, useconds;    

// Capture Filter class
class CaptureFilter : public filter
{
    public:
        // constructor
        CaptureFilter() : filter(serial_in_order)
        {           
            // open video stream with capture object
            capture = cvCaptureFromCAM(0);

            if(capture == NULL) {
                cerr << "CAMERA CAN NOT OPENED !" << endl;
                exit(EXIT_FAILURE);
            }
            else
                cout << "CAMERA OPENED." << endl;
            
            image = cvQueryFrame(capture);
            cvNamedWindow("Image");
        }
        
        // process function
        void *operator()(void *item)
        {
            gettimeofday(&start_time, NULL);
            image = cvQueryFrame(capture);
            cvShowImage("Image", image);
            cvWaitKey(1);
            
            return image;
        }
        
    private:
        CvCapture *capture;
        IplImage *image;
}; // end of First Filter class

// Sobel Filter class
class SobelFilter : public filter
{
    public:
        // constructor
        SobelFilter() : filter(serial_in_order)
        {
            image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
            sobel_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);                        
            
            cvNamedWindow("Sobel");
        }
        
        // process function
        void *operator()(void *item)
        {           
            // get image and convert it to sobel image
            image = static_cast<IplImage *>(item);            
            cvSobel(image, sobel_image, 1, 1);
            cvShowImage("Sobel", sobel_image);
            cvWaitKey(1);
            
            return image;
        }
        
    private:
        IplImage *image;
        IplImage *sobel_image;
}; // end of Sobel Filter class

// Canny Filter class
class CannyFilter : public filter
{
    public:
        // constructor
        CannyFilter() : filter(serial_in_order)
        {
            image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
            gray_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
            canny_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
            cvNamedWindow("Canny");
            sum = counter = 0;
        }
        
        void *operator()(void *item)
        {           
            image = static_cast<IplImage *>(item);
            cvCvtColor(image, gray_image, CV_RGB2GRAY);
            cvCanny(gray_image, canny_image, 50, 150);
            cvShowImage("Canny", canny_image);            
            cvWaitKey(1);
                
            gettimeofday(&end_time, NULL);

            seconds  = end_time.tv_sec  - start_time.tv_sec;
            useconds = end_time.tv_usec - start_time.tv_usec;

            mtime = ((seconds) * 1000.0 + useconds/1000.0) + 0.5;

            cout << "Elapsed time : " << mtime << " milliseconds" << endl;            
            sum += mtime;            
            cout << "TBB Average time : " << sum / ++counter << " milliseconds" << endl;
            
            return item;
        }
                
    private:
        IplImage *image;
        IplImage *gray_image;
        IplImage *canny_image;
        int counter;
        long sum;
        
}; // end of Second Filter class

#endif	/* TBBTEST_HPP */

