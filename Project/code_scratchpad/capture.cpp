#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>
#include <sys/sysinfo.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

using namespace cv;
using namespace std;
#define _GNU_SOURCE
#define RUN 0

#define VRES_ROWS (480)
#define HRES_COLS (640)

#define NUM_THREADS 1
#define NUM_CPU_CORES 1

#define IGNORED_FRAMES 10
#define NO_OF_ANALYSIS_FRAMES 50
#define NO_OF_TRANSFORMATION_FRAMES 100

unsigned char imagebuffer[1440*2560*3];


void help(){
  printf(" \n \n Usage: \n ./capture [Transform number] [IMAGE_WIDTH] [IMAGE_HEIGHT]  \n \n\
  Transform number \n 1:Sobel \n 2:Canny \n 3:Hough line \n ");
}

double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};
  clock_gettime(CLOCK_MONOTONIC, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}

Mat frame,canny_frame;
Mat gray;
vector<Vec3f> circles;

VideoCapture capture(CV_CAP_ANY);

vector<Vec4i> lines;

//variables for frame analysis
double time_before_frame_transform,time_after_frame_transform,time_taken_to_transform_frame;
double total_transformation_time;
double frame_rate;
double total_frame_count;
double reference_time;
double positive_jitter, negative_jitter;
int deadlines_miss_count=0;
int frames_required_to_be_transformed;
double deadline;


int transform_number;

typedef struct
{
    int threadIdx;
    int image_height;
    int image_width;
} threadParams_t;

    cpu_set_t threadcpu;
    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio;
    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;
    pthread_attr_t main_attr;
    pid_t mainpid;
    cpu_set_t allcpuset;


void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
     case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
     case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n"); exit(-1);
       break;
     case SCHED_RR:
           printf("Pthread Policy is SCHED_RR\n"); exit(-1);
           break;
     default:
       printf("Pthread Policy is UNKNOWN\n"); exit(-1);
   }

}



Mat hough_circle_transform_frame(Mat mat_frame){
  cvtColor(mat_frame, gray, CV_BGR2GRAY);
  GaussianBlur(gray, gray, Size(9,9), 2, 2);
  HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, gray.rows/8, 100, 50, 0, 0);

  for( size_t i = 0; i < circles.size(); i++ )
        {
          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          // circle center
          circle( mat_frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
          // circle outline
          circle( mat_frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }


  return mat_frame;
}

Mat canny_transform_frame(Mat mat_frame){
  int lowThreshold;
  int const max_lowThreshold = 100;
  int kernel_size = 3;
  int ratio=2;
  cvtColor( mat_frame, mat_frame, CV_BGR2GRAY );
  blur( mat_frame, mat_frame, Size(3,3) );
  Canny( mat_frame, mat_frame, lowThreshold, lowThreshold*ratio, kernel_size );
  return mat_frame;
}

Mat sobel_transform_frame(Mat mat_frame){
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

double get_sobel_transform_deadline(){
  int i;
  Mat analysis_frame;
  double analysis_time_before_frame_transform,analysis_time_after_frame_transform,analysis_time_taken_to_transform_frame;
  double analysis_total_transformation_time;
  double analysis_milliseconds_per_frame;
  double deadline;


  for(i=0;i<NO_OF_ANALYSIS_FRAMES+IGNORED_FRAMES;i++){
    //Ignore the first few frames
    if(i<=IGNORED_FRAMES){
           printf("\nFrame Ignored");
        continue; 
    }

    analysis_time_before_frame_transform = getTimeMsec();
    capture >> analysis_frame;
    analysis_frame = sobel_transform_frame(analysis_frame);
    analysis_time_after_frame_transform = getTimeMsec();
    analysis_time_taken_to_transform_frame = analysis_time_after_frame_transform-analysis_time_before_frame_transform;
    printf("\n analysis_time_taken_to_transform_frame : %f milliseconds",analysis_time_taken_to_transform_frame);
    analysis_total_transformation_time+=analysis_time_taken_to_transform_frame;
  }
  //printf("\n total transformation time analysis : %f",analysis_total_transformation_time);
  analysis_milliseconds_per_frame = (analysis_total_transformation_time/NO_OF_ANALYSIS_FRAMES);
  deadline = 1.2 * analysis_milliseconds_per_frame;
  printf("\n milliseconds per frame : %f \ndeadline set= %f milliseconds",analysis_milliseconds_per_frame,deadline);
  return deadline;
}


void *sobel_transform(void *threadp){
  deadline = get_sobel_transform_deadline();
  printf("\nDeadline: %f",deadline);
  while(1){
    time_before_frame_transform = getTimeMsec();  
    capture >> frame;
    frame = sobel_transform_frame(frame);
    time_after_frame_transform = getTimeMsec();
    time_taken_to_transform_frame = time_after_frame_transform-time_before_frame_transform;
    total_transformation_time+=time_taken_to_transform_frame;
    printf("\n time_taken_to_transform_frame : %f milliseconds",time_taken_to_transform_frame);
    if(deadline>time_taken_to_transform_frame){
      negative_jitter += deadline-time_taken_to_transform_frame;
    }else{
      positive_jitter += time_taken_to_transform_frame - deadline;
      printf("Deadline has been missed!");
      deadlines_miss_count++;
    }
    total_frame_count++;
    imshow("sobel_frame",frame);
    char c = cvWaitKey(1);
    if( total_frame_count > NO_OF_TRANSFORMATION_FRAMES-1){
    capture.release();
      break;
    }
  }
}



double get_canny_transform_deadline(){
  int i;
  Mat analysis_frame;
  double analysis_time_before_frame_transform,analysis_time_after_frame_transform,analysis_time_taken_to_transform_frame;
  double analysis_total_transformation_time;
  double analysis_milliseconds_per_frame;
  double deadline_local;

  for(i=0;i<NO_OF_ANALYSIS_FRAMES+IGNORED_FRAMES;i++){
    ///Ignore first few frames
      if(i<=IGNORED_FRAMES){
        continue; 
      }
    
    analysis_time_before_frame_transform = getTimeMsec();
    capture >> analysis_frame;
    analysis_frame = canny_transform_frame(analysis_frame);
    analysis_time_after_frame_transform = getTimeMsec();
    analysis_time_taken_to_transform_frame = analysis_time_after_frame_transform-analysis_time_before_frame_transform;
    printf("\n analysis_time_taken_to_transform_frame : %f milliseconds",analysis_time_taken_to_transform_frame);
    analysis_total_transformation_time+=analysis_time_taken_to_transform_frame;
  }
  //printf("\n total transformation time analysis : %f",analysis_total_transformation_time);
  analysis_milliseconds_per_frame = (analysis_total_transformation_time/NO_OF_ANALYSIS_FRAMES);
  deadline_local = 1.2 * analysis_milliseconds_per_frame;
  printf("\n milliseconds per frame : %f \ndeadline set= %f milliseconds",analysis_milliseconds_per_frame,deadline);
  return deadline_local;
}



void *canny_transform(void *threadp){
  deadline = get_canny_transform_deadline();
  printf("\nDeadline: %f",deadline);
  while(1){
    time_before_frame_transform = getTimeMsec();  
    capture >> frame;
    frame = canny_transform_frame(frame);
    time_after_frame_transform = getTimeMsec();
    time_taken_to_transform_frame = time_after_frame_transform-time_before_frame_transform;
    total_transformation_time+=time_taken_to_transform_frame;
    printf("\n time_taken_to_transform_frame : %f milliseconds",time_taken_to_transform_frame);

    if(deadline>time_taken_to_transform_frame){
      negative_jitter += deadline-time_taken_to_transform_frame;
    }else{
      positive_jitter += time_taken_to_transform_frame - deadline;
      printf("\nDeadline has been missed!");
      deadlines_miss_count++;
    }
    total_frame_count++;
    imshow("canny_frame",frame);
    char c = cvWaitKey(1);
    if( total_frame_count > NO_OF_TRANSFORMATION_FRAMES-1){
      break;
    }
  }
}

double get_hough_circle_transform_deadline(){
  int i;
  Mat analysis_frame;
  double analysis_time_before_frame_transform,analysis_time_after_frame_transform,analysis_time_taken_to_transform_frame;
  double analysis_total_transformation_time;
  double analysis_milliseconds_per_frame;
  double deadline_local;

  for(i=0;i<NO_OF_ANALYSIS_FRAMES+IGNORED_FRAMES;i++){
    ///Ignore first few frames
      if(i<=IGNORED_FRAMES){
        continue; 
      }
    
    analysis_time_before_frame_transform = getTimeMsec();
    capture >> analysis_frame;
    analysis_frame = hough_circle_transform_frame(analysis_frame);
    analysis_time_after_frame_transform = getTimeMsec();
    analysis_time_taken_to_transform_frame = analysis_time_after_frame_transform-analysis_time_before_frame_transform;
    printf("\n analysis_time_taken_to_transform_frame : %f milliseconds",analysis_time_taken_to_transform_frame);
    analysis_total_transformation_time+=analysis_time_taken_to_transform_frame;
  }
  //printf("\n total transformation time analysis : %f",analysis_total_transformation_time);
  analysis_milliseconds_per_frame = (analysis_total_transformation_time/NO_OF_ANALYSIS_FRAMES);
  deadline_local = 1.2 * analysis_milliseconds_per_frame;
  printf("\n milliseconds per frame : %f \ndeadline set= %f milliseconds",analysis_milliseconds_per_frame,deadline);
  return deadline_local;
}

void *hough_circle_transform(void *threadp){
   deadline = get_hough_circle_transform_deadline();
  printf("\nDeadline: %f",deadline);
  while(1){
    time_before_frame_transform = getTimeMsec();  
    capture >> frame;
    frame = hough_circle_transform_frame(frame);
    time_after_frame_transform = getTimeMsec();
    time_taken_to_transform_frame = time_after_frame_transform-time_before_frame_transform;
    total_transformation_time+=time_taken_to_transform_frame;
    printf("\n time_taken_to_transform_frame : %f milliseconds",time_taken_to_transform_frame);

    if(deadline>time_taken_to_transform_frame){
      negative_jitter += deadline-time_taken_to_transform_frame;
    }else{
      positive_jitter += time_taken_to_transform_frame - deadline;
      printf("\nDeadline has been missed!");
      deadlines_miss_count++;
    }
    total_frame_count++;
    imshow("canny_frame",frame);
    char c = cvWaitKey(1);
    if( total_frame_count > NO_OF_TRANSFORMATION_FRAMES-1){
      break;
    }
  }
}




void real_time_setup(){
    int i, rc, scope;
    printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

   CPU_ZERO(&allcpuset);

   for(i=0; i < NUM_CPU_CORES; i++)
       CPU_SET(i, &allcpuset);

   printf("Using CPUS=%d from total available.\n", CPU_COUNT(&allcpuset));

    mainpid=getpid();

    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=rt_max_prio;
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(rc < 0) perror("main_param");
    print_scheduler();
    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM)
      printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
      printf("PTHREAD SCOPE PROCESS\n");
    else
      printf("PTHREAD SCOPE UNKNOWN\n");

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);
    

    for(i=0; i < NUM_THREADS; i++)
    {

      CPU_ZERO(&threadcpu);
      CPU_SET(3, &threadcpu);

      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
      rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

      rt_param[i].sched_priority=rt_max_prio-i-1;
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

      threadParams[i].threadIdx=i;
    }
   
    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));    
}


void print_frame_analysis_output(){

    switch(transform_number){
      case 1:
              printf("\n*******************Sobel Transform*****************");
              break;

      case 2:
	      printf("\n*******************Canny Transform*****************");
              break;

      case 3:
	      printf("\n*******************Hough Circle Transform*****************");
              break;
    }
    printf("\nDeadline set : %f milliseconds \n",deadline);
    printf("Total Frames processed: %d \n",(int)total_frame_count);
    printf("Total transformation time: %f \n",(int)total_transformation_time);
    printf("frame rate = %f frames per second\n", frame_rate);
    printf("positive jitter = %f milliseconds\n", positive_jitter);
    printf("negative jitter = %f milliseconds\n", negative_jitter);
    printf("no of deadlines missed = %d",deadlines_miss_count);
}




int main(int argc, char** argv){
    int i=0,rc;
    setNumThreads(0);
    int image_width = 640;
    int image_height = 320;
    printf("argc=%d",argc);
    if(argc != 4 || atoi(argv[1])<1 || atoi(argv[1])>3){
      help();
      exit(1);
    }
    for (int i = 0; i < argc; ++i) 
        printf("\nargument value %d = %s\n",i,argv[i]);
    real_time_setup();
    char *sobel_string = "Sobel"; 
    transform_number = atoi(argv[1]);
    image_width = atoi(argv[2]);
    image_height = atoi(argv[3]);
    //frames_required_to_be_transformed = atoi(argv[4]);
    capture.set(CV_CAP_PROP_FRAME_WIDTH,image_width);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,image_height);

    //printf("argument 1 value %s",argv[1]);
    switch(transform_number){
    case 1:
        rc=pthread_create(&threads[0],               // pointer to thread descriptor
                      &rt_sched_attr[0],         // use specific attributes
                      //(void *)0,                 // default attributes
                      sobel_transform,                     // thread function entry point
                      (void *)0 // parameters to pass in
                     );
        break;

    case 2:
       rc=pthread_create(&threads[0],               // pointer to thread descriptor
                      &rt_sched_attr[0],         // use specific attributes
                      //(void *)0,                 // default attributes
                      canny_transform,                     // thread function entry point
                      (void *)0); // parameters to pass in
       break;
    
    case 3:
      rc=pthread_create(&threads[0],               // pointer to thread descriptor
                      &rt_sched_attr[0],         // use specific attributes
                      //(void *)0,                 // default attributes
                      hough_circle_transform,                     // thread function entry point
                      (void *)0); // parameters to pass in      
      break;
    }

    //usleep(3000000);
    /*  rc=pthread_create(&threads[2],               // pointer to thread descriptor
                      &rt_sched_attr[2],         // use specific attributes
                      //(void *)0,                 // default attributes
                      threadCounter,                     // thread function entry point
                      (void *)&(threadParams[2]) // parameters to pass in
                     );
                     */
   
   for(i=0;i<NUM_THREADS;i++){
       pthread_join(threads[i], NULL);
   }
      
    //frame_rate = (total_frame_count-100)*1000/total_transformation_time;
    frame_rate = (total_frame_count/total_transformation_time)*1000;
    print_frame_analysis_output();

   printf("\nTEST COMPLETE\n");
}
