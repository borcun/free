#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <iostream>

using namespace std;

int main()
{
	/*
	IplImage *image;// = cvLoadImage("1.jpg");
	CvCapture *capture = cvCaptureFromCAM(NULL);

	while((image = cvQueryFrame(capture)) != NULL) {
		cvNamedWindow("Image");
		cvShowImage("Image", image);
		cvWaitKey();
		cvDestroyWindow("Image");
	}

	return 0; */

	CvCapture *cap;
	int n = 0;

	while(1)
	{
		cout << "****" << endl;

		cap = cvCreateCameraCapture(n++);

	   if(cap == NULL)
		   break;

	   cvReleaseCapture(&cap);
	}

	cvReleaseCapture(&cap);

	return n-1;
}
