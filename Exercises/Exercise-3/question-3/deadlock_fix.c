#include <pthread.h>
#include <stdio.h>
#include <sched.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>


#define NUM_THREADS 2
#define THREAD_1 1
#define THREAD_2 2

typedef struct
{
    int threadIdx;
} threadParams_t;


pthread_t threads[NUM_THREADS];
threadParams_t threadParams[NUM_THREADS];

struct sched_param nrt_param;

pthread_mutex_t rsrcA = PTHREAD_MUTEX_INITIALIZER, rsrcB = PTHREAD_MUTEX_INITIALIZER;

volatile int rsrcACnt=0, rsrcBCnt=0, noWait=0;


int randomnumber = 0;
struct timespec time_state = {0,0};
struct timespec thread1_timeout={0,0};
struct timespec thread2_timeout={0,0};




double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};

  clock_gettime(CLOCK_MONOTONIC, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}

void *grabRsrcs(void *threadp)
{
   threadParams_t *threadParams = (threadParams_t *)threadp;
   int threadIdx = threadParams->threadIdx;
   int ret_thread1,ret_thread2;


   if(threadIdx == THREAD_1)
   {
     thread1_timeout.tv_sec=3;
     int thread2_was_timed_out=0;
     printf("THREAD 1 grabbing resources\n");//1
     pthread_mutex_lock(&rsrcA);
     rsrcACnt++;
     if(!noWait) usleep(1000000);
     printf("THREAD 1 got A, trying for B\n");//3
     while(pthread_mutex_trylock(&rsrcB)){
        pthread_mutex_unlock(&rsrcA);
     }

     /*
     ret_thread1 = pthread_mutex_timedlock(&rsrcB,&thread1_timeout);
      if(ret_thread1!=0){
        printf("THREAD 1 timed out! and released A\n");
        pthread_mutex_unlock(&rsrcA);
        //usleep(1000000);

        //pthread_mutex_lock(&rsrcB);
        //pthread_mutex_lock(&rsrcA);
        printf("THREAD 2 got B and A\n");
     }*/

     rsrcBCnt++;
     printf("THREAD 1 got A and B\n");//4
     pthread_mutex_unlock(&rsrcB);
     pthread_mutex_unlock(&rsrcA);
     printf("THREAD 1 done\n");//5
   }
   else
   {
     thread2_timeout.tv_sec=2;
     int thread2_was_timed_out=0;
     printf("THREAD 2 grabbing resources\n");//2
     pthread_mutex_lock(&rsrcB);
     rsrcBCnt++;
     if(!noWait) usleep(1000000);
     printf("THREAD 2 got B, trying for A\n");//5
     while(pthread_mutex_trylock(&rsrcA)){
         pthread_mutex_unlock(&rsrcB);
     }
     /*./ 
     ret_thread2=pthread_mutex_timedlock(&rsrcA,&thread2_timeout);
     if(ret_thread2!=0){
        printf("THREAD 2 timed out! and released B \n");//6
        pthread_mutex_unlock(&rsrcB);
     }
      */
      rsrcACnt++;
      printf("THREAD 2 got B and A\n");
      pthread_mutex_unlock(&rsrcA);
      pthread_mutex_unlock(&rsrcB);
      printf("THREAD 2 done\n");
     
      pthread_mutex_lock(&rsrcA);
     
   }
   pthread_exit(NULL);
}

int main (int argc, char *argv[])
{
   int rc, safe=0;
   srand(time(0));

   rsrcACnt=0, rsrcBCnt=0, noWait=0;
  
   if(argc < 2)
   {
     printf("Will set up unsafe deadlock scenario\n");
   }
   else if(argc == 2)
   {
     if(strncmp("safe", argv[1], 4) == 0)
       safe=1;
     else if(strncmp("race", argv[1], 4) == 0)
       noWait=1;
     else
       printf("Will set up unsafe deadlock scenario\n");
   }
   else
   {
     printf("Usage: deadlock [safe|race|unsafe]\n");
   }

   // Set default protocol for mutex
   pthread_mutex_init(&rsrcA, NULL);
   pthread_mutex_init(&rsrcB, NULL);

   printf("Creating thread %d\n", THREAD_1);
   threadParams[THREAD_1].threadIdx=THREAD_1;
   rc = pthread_create(&threads[0], NULL, grabRsrcs, (void *)&threadParams[THREAD_1]);
   if (rc) {printf("ERROR; pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);}
   printf("Thread 1 spawned\n");

   if(safe) // Make sure Thread 1 finishes with both resources first
   {
     if(pthread_join(threads[0], NULL) == 0)
       printf("Thread 1: %x done\n", (unsigned int)threads[0]);
     else
       perror("Thread 1");
   }

   printf("Creating thread %d\n", THREAD_2);
   threadParams[THREAD_2].threadIdx=THREAD_2;
   rc = pthread_create(&threads[1], NULL, grabRsrcs, (void *)&threadParams[THREAD_2]);
   if (rc) {printf("ERROR; pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);}
   printf("Thread 2 spawned\n");

   printf("rsrcACnt=%d, rsrcBCnt=%d\n", rsrcACnt, rsrcBCnt);
   printf("will try to join CS threads unless they deadlock\n");

   if(!safe)
   {
     if(pthread_join(threads[0], NULL) == 0)
       printf("Thread 1: %x done\n", (unsigned int)threads[0]);
     else
       perror("Thread 1");
   }

   if(pthread_join(threads[1], NULL) == 0)
     printf("Thread 2: %x done\n", (unsigned int)threads[1]);
   else
     perror("Thread 2");

   if(pthread_mutex_destroy(&rsrcA) != 0)
     perror("mutex A destroy");

   if(pthread_mutex_destroy(&rsrcB) != 0)
     perror("mutex B destroy");

   printf("All done\n");

   exit(0);
}