/*******************************************************
 Description : Detect red pixels in a video from camera
 Author		 : Burak Orcun OZKABLAN
 Number		 : 06104403
********************************************************/

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

using namespace::std;

// window features
#define X_POS 420
#define Y_POS 240
#define WIDHT 480
#define HEIGHT 320

// main function
int main(int argc, char** argv) 
{
	char ch = ' ';				      // quit character
	IplImage* frame;				  // a frame of captured CAM
	CvCapture* capture;				  // captured CAM frames
	CvFont font;					  // font which is written on screen of captured CAM
	uchar red, green, blue;			  // color variables which store colors of a pixel
	int radius = 0, counter;		  // radius of circle and counter
	int max_x, max_y, min_x, min_y;   // extreme points of circle

	// capture from camera
	capture = cvCaptureFromCAM(NULL);
	// set window name, size and coordinates
	cvNamedWindow("Red Color Detection", 0);
	cvResizeWindow("Red Color Detection",WIDHT,HEIGHT);
	cvMoveWindow("Red Color Detection",X_POS,Y_POS);
	// set font which is red area founded message
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.6, 0.6, 0, 2, CV_AA);
	
	// capture CAM frames in infinity loop
	for(;;) {
		// get one frame from camera
		frame = cvQueryFrame(capture);

		// if an error occur when get frame from camera
		// give an error message and break loop
		if(!frame) {
			cerr << "FRAME IS NULL" << endl; 					
			break;
		}

		//cout << cvGetCaptureProperty(capture, CV_CAP_PROP_FPS) << " - " << endl;
		//cvPutText(frame, "Red Color Detected !", cvPoint(10,24), &font, cvScalar(255,255,255)); 
		//cout << "FBS : " << cvGetCaptureProperty(capture,CV_CAP_PROP_FPS) << endl;
	
		// set extreme points of circle and counter
		max_x = 0;
		max_y = 0;
		min_x = X_POS + WIDHT;
		min_y = Y_POS + HEIGHT;
		counter = 0;

		// check all pixels color of a frame
		for( int y=0; y < frame->height; y++ ) {
			for( int x=0; x < frame->width; x++ ) {
				// set red, green and blue colors of a pixel
				red = (uchar)frame->imageData[y * frame->widthStep + x * frame->nChannels + 2];
				green = (uchar)frame->imageData[y * frame->widthStep + x * frame->nChannels + 1];
				blue = (uchar)frame->imageData[y * frame->widthStep + x * frame->nChannels];
				// values may be changed according to lightness,darkness of enviroment
				if(red > 100 && green < 50 && blue < 50) {
					++counter; // increase counter and count red pixels number
					
					// set maximum x value
					if(x > max_x)
						max_x = x;
					// set minimum x value
					if(min_x > x)
						min_x = x;
					// set maximum y value
					if(y > max_y)
						max_y = y;
					// set minimum x value
					if(min_y > y)
						min_y = y;
				}
			}
		}

		// if there is at least one red pixel and it is captured
		if(counter) {
			// find maximum diameter and divide by two to find radius
			if((max_x - min_x) > (max_y - min_y))
				radius = (max_x - min_x) / 2;
			else
				radius = (max_y - min_y) / 2;

			// draw circle on frame
			cvCircle(frame, cvPoint( min_x + (max_x - min_x) / 2, min_y + (max_y - min_y) / 2), radius, cvScalar(255,255,255), 2, 10);
			cvPutText(frame, "Red Color Detected !", cvPoint(10,24), &font, cvScalar(255,255,255));
			//printf("max_x : %d  max_y : %d  min_x : %d  min_y : %d  radius : %d\n",max_x,max_y,min_x,min_y,radius);
		} // end of if

		// show frame
		cvShowImage("Red Color Detection", frame);
				
		// wait 10 msec
		ch = cvWaitKey(10);
		
		// push 'q' character to quit from loop
		if(ch == 113) 
			break;
	} // end of infinity for loop

	// release capture, do not require to release IplImage pointer
	cvReleaseCapture(&capture);
	// destroy window
	cvDestroyWindow("Red Color Detection");

	return 0;
}