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

#include <queue>

using namespace cv;
using namespace std;

#define _GNU_SOURCE

#define VRES_ROWS (480)
#define HRES_COLS (640)

#define NUM_THREADS 2
#define NUM_RT_THREADS 1
#define NUM_BEST_EFFORT_THREADS 1
#define NUM_CPU_CORES 4

#define IGNORED_FRAMES 5
#define NO_OF_ANALYSIS_FRAMES 50
#define NO_OF_TRANSFORMATION_FRAMES 60

#define TEN_HZ 1

unsigned char imagebuffer[1440*2560*3];

Mat frame;
Mat empty_frame;

VideoCapture capture(0);

char *window_name = "frame";
//namedWindow(window_name, WINDOW_AUTOSIZE );
char *ffmpeg_encode_cmd = "ffmpeg -f image2 -i image_%d.ppm -vcodec mpeg4 -qscale 1 -an output_video.mp4";

//variables for frame analysis
double time_before_frame_transform,time_after_frame_transform,time_taken_to_transform_frame;
double total_transformation_time;
double frame_rate;
double total_frame_count;
double reference_time;
int deadlines_miss_count=0;
int frames_required_to_be_transformed;
double current_time;

struct timespec periodic_interval_duration = {0,0};
struct timespec start_time;
struct timespec current_deadline;

#if TEN_HZ
int time_period = 100;
#else
int time_period = 1000;
#endif

double frame_time_stamps[NO_OF_TRANSFORMATION_FRAMES];
double positive_jitter[NO_OF_TRANSFORMATION_FRAMES];
double negative_jitter[NO_OF_TRANSFORMATION_FRAMES];

queue<Mat> frame_buffer;

String image_name= "image_1";
int frame_no;



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

    pthread_t best_effort_threads[NUM_BEST_EFFORT_THREADS];
    pthread_attr_t best_effort_attr[NUM_BEST_EFFORT_THREADS];
    struct sched_param best_effort_param[NUM_BEST_EFFORT_THREADS];
    cpu_set_t best_effort_thread_cpu;


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
  #if TEN_HZ
  periodic_interval_duration.tv_nsec += 100000000;
  if(periodic_interval_duration.tv_nsec == 900000000){
     periodic_interval_duration.tv_nsec = 0;
     periodic_interval_duration.tv_sec+=1;
  }
  #else
  //periodic_interval_duration.tv_sec+=1;
  //periodic_interval_duration.tv_sec=0;
  periodic_interval_duration.tv_nsec += 999999999;
  #endif
  current_deadline.tv_sec = start_time.tv_sec + periodic_interval_duration.tv_sec;  
  current_deadline.tv_nsec = start_time.tv_nsec + periodic_interval_duration.tv_nsec;

  //current_deadline.tv_sec = periodic_interval_duration.tv_sec;
  //current_deadline.tv_nsec = periodic_interval_duration.tv_nsec;
}

void *sequencer(void *threadp){
  clock_gettime(CLOCK_MONOTONIC,&start_time);
  printf("\nstart time: %lf", convertToMsec(start_time));
  start_time.tv_nsec = 0;
  while(1){
    sem_post(&sem_capture);
    update_current_deadline();
    clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&current_deadline,NULL);
    current_time = getTimeMsec()/1000;
    syslog(LOG_INFO,"REAL TIME PROJECT TIMESTAMP: %lf", current_time);
  }
}

void *frame_capture(void *threadp){
  while(1){
    if(sem_wait(&sem_capture)!=0){
      break;
    }
    capture.read(frame);
    if(frame.empty()){ 
      printf("\nEmpty frame");
      break;
	  }
    
    putText(frame, to_string(getTimeMsec()), cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
    //(unsigned long)time(NULL)
    //imshow("frame",frame);
    
    frame_no++;
    image_name = "image_"+to_string(frame_no)+".ppm";
    frame_buffer.push(frame);
    //imshow("Empty frame",frame_buffer.front());
    //frame_buffer.pop();
    imwrite(image_name,frame);
    char c = cvWaitKey(1);
    if( c == 27 || frame_no > NO_OF_TRANSFORMATION_FRAMES){
      printf("\nSize of queue: %d",frame_buffer.size());
      capture.release();
      destroyWindow(window_name);
      sem_destroy(&sem_capture);
      break;
    }
  }
}


void *frame_grabber(void *threadp){
  while(1){ 
    usleep(1000000);
empty_queue: 
  if(frame_buffer.empty()){
    goto empty_queue;
  }
  Mat grabbed_frame = frame_buffer.front();
   frame_buffer.pop();
   imshow("grabbed frame",grabbed_frame);
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
    printf("\nMainpid=%lf",mainpid);

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
      CPU_SET(0, &threadcpu);

      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
      rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

      rt_param[i].sched_priority=rt_max_prio-i-1;
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

      threadParams[i].threadIdx=i;
    }
   
    printf("\nService threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));
    //printf("\nService threads will run on CPU core number:%d\n", CPU_GET(&threadcpu));    
}


void best_effort_setup(){
       int rc;
       CPU_ZERO(&best_effort_thread_cpu);
       CPU_SET(1, &best_effort_thread_cpu);
       CPU_SET(2, &best_effort_thread_cpu);
       CPU_SET(3, &best_effort_thread_cpu);
       rc=pthread_attr_init(&best_effort_attr[0]);
       rc=pthread_attr_setinheritsched(&best_effort_attr[0], PTHREAD_EXPLICIT_SCHED);
       rc=pthread_attr_setschedpolicy(&best_effort_attr[0], SCHED_FIFO);
       rc=pthread_attr_setaffinity_np(&best_effort_attr[0], sizeof(cpu_set_t), &best_effort_thread_cpu);
       pthread_attr_setschedparam(&best_effort_attr[0], &best_effort_param[0]);

}

int main(int argc, char** argv){
    int i=0,rc;
    real_time_setup();
    best_effort_setup();
    if(!capture.isOpened()){
      printf("\nCamera could not be found!");
    }
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

                    
     rc= pthread_create(&best_effort_threads[0],               // pointer to thread descriptor
                      &best_effort_attr[0],         // use specific attributes
                      //(void *)0,                 // default attributes
                      frame_grabber,                     // thread function entry point
                      (void *) 0 // parameters to pass in
     );
      
     
     



    //Checks if the Threads are running on their designated cores
     for( i = 0; i < NUM_THREADS; i++)
   {
       CPU_ZERO(&threadcpu);
       if(pthread_getaffinity_np(threads[i], sizeof(cpu_set_t),
       &threadcpu) == 0)
       {
           if(CPU_ISSET(0, &threadcpu) != 0)
           printf("Thread %d's Affinity: CPU 0\n", i);
           else if(CPU_ISSET(1, &threadcpu) != 0)
           printf("Thread %d's Affinity: CPU 1\n",i);
           else if(CPU_ISSET(2, &threadcpu) != 0)
           printf("Thread %d's Affinity: CPU 2\n",i);
           else if(CPU_ISSET(3, &threadcpu) != 0)
           printf("Thread %d's Affinity: CPU 3\n",i);
           else if(CPU_ISSET(4, &threadcpu) != 0)
           printf("Thread %d's Affinity: CPU 2\n",i);
           else
           printf("No affinity set for thread %d\n",i);
       }
   }

   //Check if best effort thread is not running on core 0
   if(pthread_getaffinity_np(best_effort_threads[0], sizeof(cpu_set_t),
       &best_effort_thread_cpu) == 0){
      if(CPU_ISSET(0, &threadcpu) != 0) {
        printf("Best effort thread does not run on core 0");
      } 
   }

   for(i=0;i<NUM_THREADS;i++){
       pthread_join(threads[i], NULL);
   }

   printf("\nTEST COMPLETE\n");
   system(ffmpeg_encode_cmd);

}

/*
  frame capture thread print messages 
  //printf("Running frame capture");
  time_stamps
   //time_before_frame_transform = getTimeMsec();
   time_after_frame_transform = getTimeMsec();
  time_taken_to_transform_frame = time_after_frame_transform - time_before_frame_transform;
    printf("\nTime taken to transform frame: %f",time_taken_to_transform_frame);
   
  //printf("\nframe capture running on Core %d",sched_getcpu());
  Sequencer thread print messages
  //printf("\nsequencer running on Core %d",sched_getcpu());
*/