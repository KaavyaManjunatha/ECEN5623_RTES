/*
 *
 *  Example by Sam Siewert 
 *
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;


int main( int argc, char** argv )
{
    cvNamedWindow("Capture Example", CV_WINDOW_AUTOSIZE);
    VideoCapture cap(0);
    Mat frame;

    while(1)
    {
        cap >> frame;
        if(frame.empty()){ 
	break;
	}
        imshow("Capture Example", frame);
        char c = cvWaitKey(33);
        if( c == 27 ) break;
    }
    cvDestroyWindow("Capture Example");
    
}
