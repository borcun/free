/*
 * main.cpp
 *
 *  Created on: Nov 2, 2012
 *      Author: dev01
 */

#include <tbb/task_scheduler_init.h>
#include <tbb/tbb_thread.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <iostream>

using namespace::tbb;
using namespace::std;

struct cam_att 
{
	int wcam;
	char cname;
};

char *intToS(int num)
{
	if( num == 0)
		return "0";
		
	return "1";
}

void showVideo(struct cam_att s)
{
	CvCapture *cap = cvCaptureFromCAM(s.wcam);
	
	cvNamedWindow(intToS(s.wcam));
	
	while(true) {
		cvShowImage(intToS(s.wcam), cvQueryFrame(cap));
		
		if(27 == cvWaitKey(1))
			break;
	}
	
	cvDestroyWindow(intToS(s.wcam));
}

int main()
{
	cout << "Hello TBB !" << endl;

	// Automatic startup/shutdown was not implemented because, based on Intel’s experience
	// in implementing OpenMP, we knew that parts are too problematic on some
	// operating systems to do it behind the scenes. In particular, always knowing when a
	// thread shuts down can be quite problematic.
	task_scheduler_init init(task_scheduler_init::deferred);
	int nthreads = 2;

	if(nthreads >= 1)
		init.initialize(nthreads);

	if(nthreads >= 1)
		init.terminate();
		

	struct cam_att s1, s2;
	
	s1.wcam = 0;
	s2.wcam = 1;

	tbb_thread t1(showVideo, s1);
	tbb_thread t2(showVideo, s2);
	
	t1.join();
	t2.join();
	
	return 0;
}


