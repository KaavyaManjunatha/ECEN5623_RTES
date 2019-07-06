#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <sched.h>
#include <time.h>
#include <syslog.h>

#define NUM_THREADS 2

pthread_mutex_t simple_mutex;

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
     while(1){ 
        usleep(200000);
        pthread_mutex_lock(&simple_mutex);
        sat_position_field.x++;
        sat_position_field.y++;
        sat_position_field.z++; 
        sat_position_field.roll++;
        sat_position_field.pitch++;
        sat_position_field.yaw++;
        clock_gettime(CLOCK_REALTIME,&sat_position_field.time_stamp);
        pthread_mutex_unlock(&simple_mutex);
        //usleep(1000000);
        printf("\n data after update in  update thread \n x=%lf \n y=%lf \n z=%lf \n roll=%lf \n pitch=%lf \n yaw=%lf \n time=%lf \n"\
        ,sat_position_field.x,sat_position_field.y,sat_position_field.z,sat_position_field.roll,sat_position_field.pitch,sat_position_field.yaw,convert_to_millisecond(sat_position_field.time_stamp));
        syslog(LOG_INFO,\
        "\n data after update in  update thread \n x=%lf \n y=%lf \n  z=%lf \n roll=%lf \n pitch=%lf \n yaw=%lf \n time=%lf \n"\
        ,sat_position_field.x,sat_position_field.y,sat_position_field.z,sat_position_field.roll,sat_position_field.pitch,sat_position_field.yaw,convert_to_millisecond(sat_position_field.time_stamp));
   }
}


void *read_position_field(){
    while(1){
        usleep(200000);
        pthread_mutex_lock(&simple_mutex);
        printf("\n data from read thread \n x=%lf \n y=%lf \n  z=%lf \n roll=%lf \n pitch=%lf \n yaw=%lf \n time=%lf \n"\
        ,sat_position_field.x,sat_position_field.y,sat_position_field.z,sat_position_field.roll,sat_position_field.pitch,sat_position_field.yaw,convert_to_millisecond(sat_position_field.time_stamp));
        syslog(LOG_INFO,\
        "\n data from read thread \n x=%lf \n y=%lf \n z=%lf  \n roll=%lf \n pitch=%lf \n yaw=%lf  \n time=%lf \n"\
        ,sat_position_field.x,sat_position_field.y,sat_position_field.z,sat_position_field.roll,sat_position_field.pitch,sat_position_field.yaw,convert_to_millisecond(sat_position_field.time_stamp));
        pthread_mutex_unlock(&simple_mutex);
    }
}




int main (int argc, char *argv[])
{
   int rc;
   int i=0;

   pthread_t threads[NUM_THREADS];
   pthread_attr_t thread_attribute;
   pthread_attr_t rt_sched_attr[NUM_THREADS];
 

   int rt_max_prio = sched_get_priority_max(SCHED_FIFO);
   int rt_min_prio = sched_get_priority_min(SCHED_FIFO);
   struct sched_param rt_param[NUM_THREADS];
   struct sched_param main_param;
   pthread_mutex_init(&simple_mutex, NULL);
    for(i=0; i < NUM_THREADS; i++)
    {

      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
      rt_param[i].sched_priority=rt_max_prio-i;
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);
    }

   pthread_mutex_init(&simple_mutex,NULL);

   
   //update_position_field();

    
       pthread_create(&threads[0],   // pointer to thread descriptor
                      NULL,     // use default attributes
                      update_position_field, // thread function entry point
                      NULL // parameters to pass in
                     ); 
       pthread_create(&threads[1],   // pointer to thread descriptor
                      NULL,     // use default attributes
                      read_position_field, // thread function entry point
                      NULL // parameters to pass in
                     ) ;
        for(i=0;i<NUM_THREADS;i++){
            pthread_join(threads[i], NULL);
        }
    
                 
   printf("\nTEST COMPLETE\n");
}