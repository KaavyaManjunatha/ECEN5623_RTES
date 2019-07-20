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

#define NO_OF_ANALYSIS_FRAMES 100

unsigned char imagebuffer[1440*2560*3];

static int threadRuns=0;

double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};
  clock_gettime(CLOCK_MONOTONIC, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}

Mat frame;
VideoCapture cap(0);

//variables for frame analysis
double time_before_frame_transform,time_after_frame_transform,time_taken_to_transform_frame;
double total_transformation_time;
double frame_rate;
double total_frame_count;
double reference_time;


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

Mat canny_transform(Mat mat_frame){
  int lowThreshold;
  int const max_lowThreshold = 100;
  int kernel_size = 3;
  int ratio=2;
  cvtColor( mat_frame, mat_frame, CV_BGR2GRAY );
  blur( mat_frame, mat_frame, Size(3,3) );
  Canny( mat_frame, mat_frame, lowThreshold, lowThreshold*ratio, kernel_size );
  return mat_frame;
}

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

double sobel_frame_analysis(){
  int i;
  Mat analysis_frame;
  double analysis_time_before_frame_transform,analysis_time_after_frame_transform,analysis_time_taken_to_transform_frame;
  double analysis_total_transformation_time;
  double analysis_frame_rate;
  double deadline;

  for(i=0;i<NO_OF_ANALYSIS_FRAMES;i++){
    analysis_time_after_frame_transform = getTimeMsec();
    cap >> analysis_frame;
    analysis_frame = sobel_transform(analysis_frame);
    analysis_time_after_frame_transform = getTimeMsec();
    analysis_time_taken_to_transform_frame = analysis_time_after_frame_transform-analysis_time_before_frame_transform;
    analysis_total_transformation_time+=analysis_time_taken_to_transform_frame;
  }
  frame_rate = (total_frame_count/total_transformation_time);
  deadline = 1.2 *frame_rate;
  return deadline;
}



void *sobel_transform(void *threadp){
  double deadline = sobel_frame_analysis();
  printf("Deadline: %f",deadline);
  while(1){
    time_before_frame_transform = getTimeMsec();
    cap >> frame;
    frame = sobel_transform(frame);
    time_after_frame_transform = getTimeMsec();
    time_taken_to_transform_frame = time_after_frame_transform-time_before_frame_transform;
    total_transformation_time+=time_taken_to_transform_frame;
    total_frame_count++;
    imshow("sobel_frame",frame);
    char c = cvWaitKey(10);
    if( c == 27 ){
      break;
    }
  }
}



void *canny_transform(void *threadp){
  while(1){
    time_before_frame_transform = getTimeMsec();
    cap >> frame;
    frame = canny_transform(frame);
    time_after_frame_transform = getTimeMsec();
    time_taken_to_transform_frame = time_after_frame_transform-time_before_frame_transform;
    total_transformation_time+=time_taken_to_transform_frame;
    total_frame_count++;
    imshow("canny_frame",frame);
    char c = cvWaitKey(10);
    if( c == 27 ){
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





void *threadCounter(void *threadp){
  threadParams_t *threadParams = (threadParams_t *)threadp;
  print_scheduler();
  threadRuns++;
  printf("\n Running thread %d",threadParams->threadIdx);
}

int main(int argc, char** argv){
    int i=0,rc;
    for (int i = 0; i < argc; ++i) 
        printf("\nargument value %d = %s\n",i,argv[i]);
    real_time_setup();
    char *sobel_string = "Sobel"; 
    int transform_number = atoi(argv[1]);
    //printf("argument 1 value %s",argv[1]);
    if(transform_number==1){
    
    rc=pthread_create(&threads[0],               // pointer to thread descriptor
                      &rt_sched_attr[0],         // use specific attributes
                      //(void *)0,                 // default attributes
                      sobel_transform,                     // thread function entry point
                      (void *)0 // parameters to pass in
                     );

    }else if(transform_number==2){
       rc=pthread_create(&threads[0],               // pointer to thread descriptor
                      &rt_sched_attr[0],         // use specific attributes
                      //(void *)0,                 // default attributes
                      canny_transform,                     // thread function entry point
                      (void *)0); // parameters to pass in
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
    printf("Total Frames processed: %d \n",(int)total_frame_count);
    printf("Total transformation time: %f \n",(int)total_transformation_time);
    printf("frame rate = %f frames per second\n", frame_rate);

   printf("\nTEST COMPLETE\n");
}