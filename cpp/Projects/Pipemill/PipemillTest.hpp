/**
 * @file:    PipemillTest.hpp
 * @author:  boo
 * @title:   Pipemill Test Class
 * @version: 1.0
 * @date:    January 25, 2013, 9:37 AM
 * @cond:    Connected USB2.0 Camera
 */

#ifndef PIPEMILLTEST__HPP
#define	PIPEMILLTEST__HPP

#include "OutConnector.cpp"
#include "InConnector.cpp"
#include "InputFilter.hpp"
#include "ProcessFilter.hpp"
#include "MultiPipeline.hpp"
#include "OutputFilter.hpp"
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <iostream>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

using namespace Pipemill;

struct timeval start_time, end_time;
long mtime;
long seconds, useconds;    

// Capture Filter class
class CaptureFilter : public InputFilter
{
    public:
        /// \brief constructor
        CaptureFilter() : InputFilter(InputFilter::serial_in_order)
        {
            // open video stream with capture object
            if(!(capture = cvCaptureFromCAM(0))) {
                std::cerr << "CAMERA CAN NOT OPENED !" << std::endl;
                exit(EXIT_FAILURE);
            }

            std::cout << "CAMERA OPENED." << std::endl;
            // grab frame from capture
            image = cvQueryFrame(capture);
            // create an out connector object as many as 1 image buffer
            m_oc = new OutConnector<IplImage>(image->width * image->height * image->nChannels);
            // create a window for video stream
            cvNamedWindow("Image");
        }

        /// \brief destructor
        ~CaptureFilter()
        {
            cvReleaseCapture(&capture);
            cvReleaseImage(&image);
        }
        
        /// \brief process function
        /// @param index - index is not important for input filter class
        /// @return -
        void process(void *index)
        {
            // get time
            gettimeofday(&start_time, NULL);
            // grab frame from capture
            image = cvQueryFrame(capture);
            // write image data into buffer of out connector
            m_oc->writeData(image);
            // show image
            cvShowImage("Image", image);

            if(27 == cvWaitKey(1)) {
                cvDestroyAllWindows();
                exit(EXIT_SUCCESS);
            }
        }
        
        /// \brief out connector object
        OutConnector<IplImage> *m_oc;
        
    private:
        CvCapture *capture;
        IplImage *image;

}; // end of First Filter class

// Sobel Filter class
class SobelFilter : public ProcessFilter
{
    public:
        /// \brief constructor
        SobelFilter() : ProcessFilter(BaseFilter::serial_in_order)
        {
            // create two image from capturing and sobel filter
            image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
            sobel_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
            // create an in connector object to connect to out connector of previous filter
            m_in = new InConnector<IplImage>();
            // create an out connector object to connect to in connector of next filter
            m_oc = new OutConnector<IplImage>(image->width * image->height * image->nChannels);
            // create a window for sobel video stream
            cvNamedWindow("Sobel");
        }
        
        /// \brief destructor
        ~SobelFilter()
        {
            cvReleaseImage(&image);
            cvReleaseImage(&sobel_image);
        }

        /// \brief process function
        /// @param index - index is so important for process filter class. It is order of data from previous filter.
        /// @return -
        void process(void *index)
        {           
            // get image from previous filter
            // readData function of in connector object return a vector object.
            // this vector is a line of shared buffer of out connector and in connector.
            // In every step, value in index is assigned to vector and vector is returned.
            // If there is connection of one in connector - one out connector, vector size equals 1.
            // If there are connections of one in connector - many out connector, vector size is bigger than 1.
            *image = m_in->readData(index).at(0);

            // convert RGB image to sobel image
            cvSobel(image, sobel_image, 1, 1);
            cvShowImage("Sobel", sobel_image);
            cvWaitKey(1);

            // send image from previous filter(CaptureFilter) to next filter(CannyFilter)
            m_oc->writeData(image);
        }
        
        /// \brief in connector object
        InConnector<IplImage> *m_in;
        /// \brief out connector object
        OutConnector<IplImage> *m_oc;
        
    private:
        IplImage *image;
        IplImage *sobel_image;

}; // end of Sobel Filter class

// Canny Filter class
class CannyFilter : public ProcessFilter
{
    public:
        /// \brief constructor
        CannyFilter() : ProcessFilter(BaseFilter::serial_in_order)
        {
            // create three images for capturing and edge detection process
            image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
            gray_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
            canny_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
            // create just one in connector object
            m_in = new InConnector<IplImage>();
            // create a window for edge detection process
            cvNamedWindow("Canny");
            // zero sum and counter variable
            sum = counter = 0;
        }

        /// \brief destructor
        ~CannyFilter()
        {
            cvReleaseImage(&image);
            cvReleaseImage(&gray_image);
            cvReleaseImage(&canny_image);
        }

        /// \brief process function
        /// @param index - index is so important for out filter class as process filter class. It is order of data from previous filter.
        /// @return -
        void process(void *item)
        {           
            // get data from previous filter
            *image = m_in->readData(item).at(0);
            // convert image to gray image
            cvCvtColor(image, gray_image, CV_RGB2GRAY);
            // run Canny algorithm on gray image
            cvCanny(gray_image, canny_image, 50, 150);
            // show results
            cvShowImage("Canny", canny_image);
            cvWaitKey(1);
            
            // end time that starts in CaptureFilter Class
            gettimeofday(&end_time, NULL);
            // evaluate elapsed time between first process and last process
            seconds  = end_time.tv_sec  - start_time.tv_sec;
            useconds = end_time.tv_usec - start_time.tv_usec;
            mtime = ((seconds) * 1000.0 + useconds / 1000.0) + 0.5;
            std::cout << "Elapsed time : " << mtime << "milliseconds" << std::endl;
            // evaluate average time
            sum += mtime;            
            std::cout << "Pipemill Average time : " << sum / ++counter << " milliseconds" << std::endl;
        }
        
        /// \brief in connector object
        InConnector<IplImage> *m_in;
        
    private:
        IplImage *image;
        IplImage *gray_image;
        IplImage *canny_image;
        int counter;
        long sum;
        
}; // end of Second Filter class

#endif	/* PIPEMILLTEST__HPP */

