#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <sched.h>
#include <time.h>
#include <syslog.h>

#define NUM_THREADS 2


pthread_mutex_t simple_mutex;

int ret1;

typedef struct
{
    int threadIdx;
} threadParams_t;

typedef struct{
    struct timespec time_stamp;
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;

}position_field;

static position_field sat_position_field= {{0,0},0,0,0,0,0};
static position_field previous_sat_position_field= {{0,0},0,0,0,0,0};
static struct timespec lock_duration = {1,0};

pthread_t threads[NUM_THREADS];
threadParams_t threadParams[NUM_THREADS];

double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};

  clock_gettime(CLOCK_MONOTONIC, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}

double convert_to_millisecond(struct timespec time_stamp){
    return ((time_stamp.tv_sec*1000.0)+(time_stamp.tv_nsec/1000000.0));
}


void *update_position_field(){
    pthread_mutex_lock(&simple_mutex);
    sat_position_field.x++;
    sat_position_field.y++;
    sat_position_field.z++; 
    //usleep(100000000000);
    sat_position_field.roll++;
    sat_position_field.pitch++;
    sat_position_field.yaw++;
    clock_gettime(CLOCK_REALTIME,&sat_position_field.time_stamp);
    pthread_mutex_unlock(&simple_mutex);
    syslog(LOG_INFO,\
    "\n data after update in  update thread \n x=%lf \n y=%lf \n  z=%lf \n roll=%lf \n pitch=%lf \n yaw=%lf \n time=%lf "\
    ,sat_position_field.x,sat_position_field.y,sat_position_field.z,sat_position_field.roll,sat_position_field.pitch,sat_position_field.yaw,convert_to_millisecond(sat_position_field.time_stamp));
    return;
}


void *read_position_field(){
     int rc=pthread_mutex_timedlock(&simple_mutex,&lock_duration);
     if(rc!=0){
         syslog(LOG_INFO,"No new data available");
         pthread_exit(&ret1);
     }
    syslog(LOG_INFO,\
    "\n data from read thread \n x=%lf \n y=%lf \n z=%lf  \n roll=%lf \n pitch=%lf \n yaw=%lf  \n time=%lf "\
    ,sat_position_field.x,sat_position_field.y,sat_position_field.z,sat_position_field.roll,sat_position_field.pitch,sat_position_field.yaw,convert_to_millisecond(sat_position_field.time_stamp));
     pthread_mutex_unlock(&simple_mutex);
    return;
}

void *timed_read_position(){
    pthread_mutex_lock(&simple_mutex);

}




int main (int argc, char *argv[])
{
   int rc;
   int i=0;
   pthread_mutex_init(&simple_mutex, NULL);


    for(i=0;i<NUM_THREADS;i++){
        threadParams[i].threadIdx=i;  usleep(1000000);
        }
  
   printf("\n***Before update ***");

   pthread_mutex_init(&simple_mutex,NULL);
   
   //update_position_field();
    while(1){

    
       pthread_create(&threads[0],   // pointer to thread descriptor
                      NULL,     // use default attributes
                      update_position_field, // thread function entry point
                      (void *)&(threadParams[i]) // parameters to pass in
                     ); 
       pthread_create(&threads[1],   // pointer to thread descriptor
                      NULL,     // use default attributes
                      read_position_field, // thread function entry point
                      (void *)&(threadParams[i]) // parameters to pass in
                     );
                     /* 
        pthread_create(&threads[2],   // pointer to thread descriptor
                      NULL,     // use default attributes
                      read_position_field, // thread function entry point
                      (void *)&(threadParams[i]) // parameters to pass in
                     );             
                    */
        

        for(i=0;i<NUM_THREADS;i++){
            pthread_join(threads[i], NULL);
        }
    }
                 
   printf("\nTEST COMPLETE\n");
}