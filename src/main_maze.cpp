#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <X11/keysym.h>

int main(int argc, char **argv)
{
	/* Start the CV system and get the first v4l camera */
	cvInitSystem(argc, argv);
	CvCapture *cam = cvCreateCameraCapture(0);

	/* Create a window to use for displaying the images */
	cvNamedWindow("img", 0);
	cvMoveWindow("img", 200, 200);

	/* Display images until the user presses q */
	while (1) {
		cvGrabFrame(cam);
		IplImage *img = cvRetrieveFrame(cam);
		cvShowImage("img", img);
		if (cvWaitKey(10) == XK_q)
			return 0;
		cvReleaseImage(&img);
	}
}