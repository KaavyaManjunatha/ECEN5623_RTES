#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include<pthread.h>

//#include <iostream>

//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include "opencv2/imgproc/imgproc.hpp"

//using namespace std;
#define _GNU_SOURCE
#define RUN 0

#define VRES_ROWS (480)
#define HRES_COLS (640)

#define NUM_THREADS 2

unsigned char imagebuffer[1440*2560*3];


typedef struct
{
    int threadIdx;
} threadParams_t;

double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};
  clock_gettime(CLOCK_MONOTONIC, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}

void *threadCounter(void *threadp){
  threadParams_t *threadParams = (threadParams_t *)threadp;
  printf("\n Running thread %d",threadParams->threadIdx);
}

int main( int argc, char** argv )
{
    int i,rc,j;
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
    printf("\nstage 1");

//CPU affinity setting  
  CPU_ZERO(&threadcpu);
  CPU_SET(3, &threadcpu);
  rt_max_prio = sched_get_priority_max(SCHED_FIFO);
  rt_min_prio = sched_get_priority_min(SCHED_FIFO);
  for(j=0;j<1;j++){

    for(i=0;i<NUM_THREADS;i++){
    CPU_ZERO(&threadcpu);
    CPU_SET(3, &threadcpu);
       
      rc=pthread_attr_init(&rt_sched_attr[i]);
      if(rc!=0){
      printf("\nError after  pthread attributes setting");
    }

      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      if(rc!=0){
      printf("\nError after  pthread attributes setting");
    }

      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
      if(rc!=0){
      printf("\nError after  pthread attributes setting");
    }

      rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);
      if(rc!=0){
      printf("\nError after  pthread attributes setting");
    }

      rt_param[i].sched_priority=rt_max_prio-i;
      if(rc!=0){
      printf("\nError after  pthread attributes setting");
    }

      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);
      threadParams[i].threadIdx=i;
      printf("\n pthread attribute  setting code was executed");
    }


     if(rc!=0){
      printf("\nError after  pthread attributes setting");
    }

    for(i=0;i<NUM_THREADS;i++){
      rc = pthread_create(&threads[i],&rt_sched_attr[i],threadCounter,(void *)&threadParams[i]);
      printf("\n rc value after pthread create %d = %d",i,rc);
    }

    if(rc!=0){
      printf("\nError in pthread create");
    }

    for(i=0;i<NUM_THREADS;i++){
      pthread_join(threads[i],NULL);
    }
  }
}