#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>
#include <sys/sysinfo.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <syslog.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

using namespace cv;
using namespace std;

#define _GNU_SOURCE

#define VRES_ROWS (480)
#define HRES_COLS (640)

#define NUM_THREADS 2
#define NUM_RT_THREADS 1
#define NUM_BE_THREADS 1
#define NUM_CPU_CORES 1

#define IGNORED_FRAMES 5
#define NO_OF_ANALYSIS_FRAMES 50
#define NO_OF_TRANSFORMATION_FRAMES 20

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

double convertToMsec(struct timespec event_ts){
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}

Mat frame;

VideoCapture capture(CV_CAP_ANY);

//variables for frame analysis
double time_before_frame_transform,time_after_frame_transform,time_taken_to_transform_frame;
double total_transformation_time;
double frame_rate;
double total_frame_count;
double reference_time;
double positive_jitter, negative_jitter;
int deadlines_miss_count=0;
int frames_required_to_be_transformed;
double current_time;

struct timespec periodic_interval_duration = {0,0};
struct timespec start_time;
struct timespec current_deadline;

String image_name= "image_1";
int frame_no;

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

    sem_t sem_capture;

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

void update_current_deadline(){
  periodic_interval_duration.tv_sec+=1;
  current_deadline.tv_sec = start_time.tv_sec + periodic_interval_duration.tv_sec;
  current_deadline.tv_nsec = start_time.tv_nsec + periodic_interval_duration.tv_nsec;
}

void *sequencer(void *threadp){
  clock_gettime(CLOCK_MONOTONIC,&start_time);
  printf("\nstart time: %lf", convertToMsec(start_time));
  while(1){
  //sempost service 1 
  //sleep and wake up every second
    sem_post(&sem_capture);
    update_current_deadline();
    clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&current_deadline,NULL);
    current_time = getTimeMsec()/1000;
    syslog(LOG_INFO,"REAL TIME PROJECT TIMESTAMP: %lf", current_time);
  }
}

void *frame_capture(void *threadp){
  while(1){
    sem_wait(&sem_capture);
    //time_before_frame_transform = getTimeMsec();
    capture >> frame;
    if(frame.empty()){ 
	    break;
	  }
    //time_after_frame_transform = getTimeMsec();
    //time_taken_to_transform_frame = time_after_frame_transform - time_before_frame_transform;
    //printf("\nTime taken to transform frame: %f",time_taken_to_transform_frame);
    imshow("frame",frame);
    frame_no++;
    image_name = "image_"+to_string(frame_no)+".ppm";
    imwrite(image_name,frame);
    char c = cvWaitKey(1);
    if( c == 27 || frame_no > NO_OF_TRANSFORMATION_FRAMES){
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


    rc = sem_init(&sem_capture,0,1);

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

int main(int argc, char** argv){
    int i=0,rc;
    real_time_setup();
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);

     rc=pthread_create(&threads[0],               // pointer to thread descriptor
                      &rt_sched_attr[0],         // use specific attributes
                      //(void *)0,                 // default attributes
                      sequencer,                     // thread function entry point
                      (void *)0 // parameters to pass in
                     );
                     
     rc= pthread_create(&threads[1],               // pointer to thread descriptor
                      &rt_sched_attr[1],         // use specific attributes
                      //(void *)0,                 // default attributes
                      frame_capture,                     // thread function entry point
                      (void *) 0 // parameters to pass in
     );

   for(i=0;i<NUM_THREADS;i++){
       pthread_join(threads[i], NULL);
   }
   printf("\nTEST COMPLETE\n");
}



