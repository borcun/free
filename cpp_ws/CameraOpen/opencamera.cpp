#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

// function that display usage information
void usage();

int main(int argc, char **argv)
{
  if(argc != 2) {
    usage();
    return -1;
  }

  const int cam_id = atoi(argv[1]);
  CvCapture *capture = cvCaptureFromCAM(cam_id);
  char quit = 'c';

  if(NULL == capture) {
    std::cerr << "Camera not opened" << std::endl;
    return -1;
  }

  std::cout << "Press q to quit" << std::endl;

  cvNamedWindow(argv[1]);

  while(quit != 'q') {
    cvShowImage(argv[1], cvQueryFrame(capture));
    quit = cvWaitKey(1);
  }

  cvDestroyAllWindows();
  cvReleaseCapture(&capture);

  return 0;
}

void usage()
{
  std::cout << "./opencam <cam id>" << std::endl;
  return;
}
