#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

#define VRES_ROWS (480)
#define HRES_COLS (640)

unsigned char imagebuffer[1440*2560*3];

Mat sobel_transform(Mat mat_frame){
  //printf("Performing sobel transform");
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  GaussianBlur( mat_frame, mat_frame, Size(3,3), 0, 0, BORDER_DEFAULT );
  cvtColor( mat_frame, mat_frame, CV_RGB2GRAY );
  Mat grad_x, grad_y;
  Mat abs_grad_x, abs_grad_y;

  Sobel( mat_frame, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );   
  convertScaleAbs( grad_x, abs_grad_x );

  Sobel( mat_frame, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );   
  convertScaleAbs( grad_y, abs_grad_y );

  addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, mat_frame );

  return mat_frame;
}

double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};
  clock_gettime(CLOCK_MONOTONIC, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}

int main( int argc, char** argv )
{
    cvNamedWindow("Capture Example", CV_WINDOW_AUTOSIZE);
    //cvNamedWindow("binary", CV_WINDOW_AUTOSIZE);
    double frameCount;
    double transform_frame_rate;
    VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,HRES_COLS);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,VRES_ROWS);
    Mat mat_frame;
    double time_before_transform, time_after_transform, time_taken_to_transform_frame, total_transform_time;


    while(1)
    {
        cap >> mat_frame;
        if(mat_frame.empty()){
            break;
        }
        frameCount++;
        time_before_transform = getTimeMsec();
        mat_frame = sobel_transform(mat_frame);
        time_taken_to_transform_frame = (time_after_transform-time_before_transform);
        total_transform_time+=time_taken_to_transform_frame;
        imshow("Capture Example", mat_frame);
        char c = cvWaitKey(33);
        if( c == 27 ){
             break;
        }    
    }    
    transform_frame_rate = (frameCount/total_transform_time)*100;
    printf("\nTotal transformation time %lf",total_transform_time);
    printf("\n Total Frames processed %lf",frameCount);
    printf("\ntransformation time per frame %lf\n",transform_frame_rate);
    cvDestroyWindow("Capture Example");   
}