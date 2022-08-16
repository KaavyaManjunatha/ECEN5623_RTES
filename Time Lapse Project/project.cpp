/*
    Real time-lapse image capture system
    RT Services
    Service 1 - Sequencer - dispatches the RT services in the specified frequency
    Service 2 - Acquires frames from the camera

    Best Efforts Services
    frame_grabber - Grabs the frame from the queue and saves it
                  - Applied sobel transform and saves the original image as well 
*/

//edited test
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

#include <syslog.h>
#include <sys/sysinfo.h>
#include <sys/time.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>
#include <mqueue.h>
#include <signal.h>
#include <iostream>

#include <errno.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <errno.h>
#include <queue>

using namespace std;
using namespace cv;

#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (1)
#define TRUE (1)
#define FALSE (0)

#define NUM_THREADS (1+1)
#define NUM_BEST_EFFORT_THREADS 1
#define NUM_FRAMES_REQUIRED 1800

#define HRES 640
#define VRES 480

#define USE_SERVICE_2 0
#define ABS_SLEEP 1
#define NUM_CYCLES 1800 + 18

#define TEN_HZ 1
#define K 4.0
#define NO_IMAGE_STRUCTURE 0

double PSF[9] = {-K/8.0, -K/8.0, -K/8.0, -K/8.0, K+1.0, -K/8.0, -K/8.0, -K/8.0, -K/8.0};

int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE;
sem_t semS1, semS2, semS3, semS4, semS5, semS6, semS7;
int sem1_count;


VideoCapture capture(0);

struct timespec remaining_time;

typedef struct
{
    int threadIdx;
    unsigned long long sequencePeriods;
} threadParams_t;

typedef struct{
    Mat frame;
    double time_stamp;

}image_t;

queue<Mat> frame_buffer;
queue<image_t> image_buffer;
image_t image_object; 

bool run_sobel;
bool run_10_hz;

Mat frame;
unsigned long long frame_count;

char image_name[50] = "frame_1.ppm";
char grabbed_image_name[50] = "grabbed_frame_1.ppm";

char *ffmpeg_encode_cmd = "ffmpeg -f image2 -i image_%d.ppm -vcodec mpeg4 -qscale 1 -an output_video.mp4";

//Time variables
struct timeval  start_time_val;
struct timespec current_deadline;
struct timespec periodic_interval_duration = {0,0};
struct timespec start_time = {0,0};

double frame_time_stamps[NUM_FRAMES_REQUIRED];
double frame_jitter[NUM_FRAMES_REQUIRED];
double positive_jitter[NUM_FRAMES_REQUIRED];
double negative_jitter[NUM_FRAMES_REQUIRED];
double total_jitter;

#if TEN_HZ 
double time_period = 100;
#else
double time_period = 1000;
#endif


/* -------------------------------
    Time stamp variables for logging 
    and analysis
------------------------------- */ 
//sequencer millisecond variables
double sequencer_start_time;
double sequencer_current_time;
double sequencer_before_sempost;
double sequencer_after_sempost;

double sequencer_total_execution_time;
double sequencer_average_execution_time;
double sequencer_worst_case_time;


//frame capture time stamps
double service_1_initialization_time;
double service_1_ref_time;
double service_1_start_time;
double service_1_end_time;
double before_capture;
double after_capture;
double service_2_start_time;
double service_2_end_time;

double service_1_average_execution_time;
double service_1_worst_case_time;
double service_1_total_time;

//RT thread variables
    int i, rc, scope;
    struct timeval current_time_val;
    cpu_set_t allcpuset;
    cpu_set_t threadcpu;
    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio;
    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;
    pthread_attr_t main_attr;
    pid_t mainpid;

//Best Effort Thread setup variables 
    cpu_set_t best_effort_thread_cpu;
    pthread_t best_effort_threads[NUM_BEST_EFFORT_THREADS];
    pthread_attr_t best_effort_attr[NUM_BEST_EFFORT_THREADS];
    struct sched_param best_effort_param[NUM_BEST_EFFORT_THREADS];

//Real Time Thread service declarations
void *Sequencer(void *threadp);
void *Service_1(void *threadp); 
void *Service_2(void *threadp);

//RT Thread function declarations
void capture_frame();

//Best Effort threads
void *grab_frame(void *threadp);

//other supporting function declaration
void RT_threads_setup(void);
void best_effort_thread_setup(void);
void camera_setup(void);
void update_current_deadline(void);
double getTimeMsec(void);
void print_scheduler(void);
void sharpen_frame();
void custom_sharpen();
void best_effort_custom_sharpen_frame(Mat frame);

int main(int argc,char **argv)
{
    int log_options = LOG_PID;
    openlog("REAL TIME PROJECT:",log_options,LOG_CRIT);
    RT_threads_setup();
    //best_effort_thread_setup();
    camera_setup();
    string sobel = "Sobel";
    printf("Starting Sequencer Demo\n");
    gettimeofday(&start_time_val, (struct timezone *)0);
    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Sequencer @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    if(argc > 1){
        string sobel_argument(argv[1]);
        if(sobel_argument.compare(sobel)==0)
        run_sobel = true;    
    } 


    // Wait for service threads to initialize and await relese by sequencer.
    // Note that the sleep is not necessary of RT service threads are created wtih 
    // correct POSIX SCHED_FIFO priorities compared to non-RT priority of this main
    // program.
    // usleep(1000000);
    // Create Sequencer thread, which like a cyclic executive, is highest prio
    printf("Start sequencer\n");
    threadParams[0].sequencePeriods=NUM_CYCLES;

    // Sequencer = RT_MAX	@ 30 Hz
    //rt_param[0].sched_priority=rt_max_prio;
    //pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc=pthread_create(&threads[0], &rt_sched_attr[0], Sequencer, (void *)&(threadParams[0]));
    if(rc < 0)
        perror("pthread_create for sequencer service 0");
    else
        printf("pthread_create successful for sequeencer service 0\n");


    // Create Service threads which will block awaiting release for:
    
    // Servcie_1 = RT_MAX-1	@ 3 Hz
    //
    //rt_param[1].sched_priority=rt_max_prio-1;
    //pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
    rc=pthread_create(&threads[1],               // pointer to thread descriptor
                      &rt_sched_attr[1],         // use specific attributes
                      //(void *)0,               // default attributes
                      Service_1,                 // thread function entry point
                      (void *)&(threadParams[1]) // parameters to pass in
                     );
    if(rc < 0)
        perror("pthread_create for service 1");
    else
        printf("pthread_create successful for service 1\n");


    // Service_2 = RT_MAX-2	@ 1 Hz
    //rt_param[2].sched_priority=rt_max_prio-2;
    //(&rt_sched_attr[2], &rt_param[2]);
    #if USE_SERVICE_2
    rc=pthread_create(&threads[2], &rt_sched_attr[2], Service_2, (void *)&(threadParams[2]));
    if(rc < 0)
        perror("pthread_create for service 2");
    else
        printf("pthread_create successful for service 2\n");
    #endif
    //Best effort Service creations 
        rc= pthread_create(&best_effort_threads[0],               // pointer to thread descriptor
                      &best_effort_attr[0],         // use specific attributes
                      //(void *)0,                 // default attributes
                      grab_frame,                     // thread function entry point
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

   //Check to see if best effort threads are not running on Core 0 which is dedicated to Real Time threads 
  if(pthread_getaffinity_np(best_effort_threads[0], sizeof(cpu_set_t),
       &best_effort_thread_cpu) == 0){
      if(CPU_ISSET(0, &threadcpu) != 0) {
        printf("Best effort thread does not run on core 0\n");
      } 
       }
    
   for(i=0;i<NUM_THREADS;i++)
       pthread_join(threads[i], NULL);
       
    return 0;

   printf("\nTEST COMPLETE\n");
}

void *Sequencer(void *threadp)
{

    sequencer_start_time = getTimeMsec();
    clock_gettime(CLOCK_REALTIME, &start_time);
    struct timespec delay_time = {0,33333333}; // delay for 33.33 msec, 30 Hz
    double current_time;
    double residual;
    int rc, delay_cnt=0;
    unsigned long long seqCnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    bool start_flag = 0;
    char keypress='a',c = 'b';
    //clock_gettime(CLOCK_MONOTONIC, &event_ts);
    syslog(LOG_CRIT, "Sequencer thread @ %lf milliseconds",sequencer_start_time);
    printf("Sequencer thread @ %lf milliseconds",sequencer_start_time);
            
        char input;
        cout<<"\nPress t followed by enter to trigger sequencer";
        cin>>input;    
     do
    {

        #if ABS_SLEEP
        update_current_deadline();
        //rc=clock_nanosleep(CLOCK_REALTIME,TIMER_ABSTIME,&current_deadline,NULL);
        do
        {
            rc=clock_nanosleep(CLOCK_REALTIME,TIMER_ABSTIME,&current_deadline,NULL);
             if(rc == EINTR)
            { 
                residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC);
                if(residual > 0.0) printf("residual=%lf, sec=%d, nsec=%d\n", residual, (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);
                delay_cnt++;
            }

            else if(rc < 0)
            {
                perror("Sequencer nanosleep");
                exit(-1);
            }           
        } while((residual > 0.0) && (delay_cnt < 100));

        #else

         do
         {
             rc=nanosleep(&delay_time, &remaining_time);

                if(rc == EINTR)
                { 
                    residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC);

                    if(residual > 0.0) printf("residual=%lf, sec=%d, nsec=%d\n", residual, (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);
 
                    delay_cnt++;
             }
                else if(rc < 0)
                {
                    perror("Sequencer nanosleep");
                    exit(-1);
                }
           
            }while((residual > 0.0) && (delay_cnt < 100));

        #endif


        while(!start_flag){
            if(input == 't'){
                printf("Sequencer triggered!\n");
                start_flag = true;
                break;
            }
        }


        sequencer_before_sempost = getTimeMsec();    


        if(sem_getvalue(&semS1,&sem1_count)!=0){
                printf("Error in sem_getvalue\n");
        }

        if(sem1_count > 0){
            sem_trywait(&semS1);
        }
        syslog(LOG_CRIT,"\nSemaphore value in cycle %llu: %d",seqCnt,sem1_count);
        seqCnt++;
        sequencer_current_time = getTimeMsec();
        syslog(LOG_CRIT, "Sequencer cycle %llu @ %lf milliseconds\n", seqCnt,(sequencer_current_time-sequencer_start_time));
        // Release each service at a sub-rate of the generic sequencer rate
        #if ABS_SLEEP
        // Service_1 = RT_MAX-1	@ 3 Hz
        if((seqCnt % 1) == 0) sem_post(&semS1);

        #else
        if((seqCnt % 30) == 0) sem_post(&semS1);
        // Service_2 = RT_MAX-2	@ 1 Hz
        //if((seqCnt % 1) == 0) sem_post(&semS2
        #endif

        sequencer_after_sempost = getTimeMsec();
        sequencer_total_execution_time += sequencer_after_sempost-sequencer_before_sempost;
        syslog(LOG_CRIT, "Sequencer cycle %llu execution time  %lf milliseconds\n", seqCnt,(sequencer_after_sempost-sequencer_before_sempost));
        syslog(LOG_CRIT,"Sequencer average execution time: %lf", sequencer_total_execution_time);

    } while(!abortTest && (seqCnt < threadParams->sequencePeriods));
    abortS1=TRUE;
    sem_post(&semS1); //sem_post(&semS2); 
     //abortS2=TRUE; 
    syslog(LOG_CRIT,"Sequencer average execution time: %lf", sequencer_average_execution_time);
    printf("Sequencer average execution time: %lf", sequencer_average_execution_time);
    //printf("Service 1 total jitter: %lf", total_jitter);
    
    pthread_exit((void *)0);
}

void *Service_1(void *threadp)
{
    struct timeval current_time_val;
    double current_time;
    unsigned long long S1Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    service_1_initialization_time = getTimeMsec();
    syslog(LOG_CRIT, "Service 1 intialization time: %lf \n", service_1_initialization_time);
    printf("Service 1 initilization time: %lf \n", service_1_ref_time);
    while(!abortS1)
    {
        sem_wait(&semS1);
        if(S1Cnt==0){
                service_1_ref_time = getTimeMsec();

        }
        service_1_start_time = getTimeMsec();
        frame_jitter[S1Cnt] = (service_1_start_time - service_1_ref_time) - (S1Cnt*time_period);
        syslog(LOG_CRIT,"jitter? %lf",(service_1_start_time - service_1_ref_time) - (S1Cnt*time_period));
        total_jitter += (service_1_start_time - service_1_ref_time) - (S1Cnt*time_period);    
        S1Cnt++;
        capture_frame();
        service_1_end_time = getTimeMsec();
        service_1_total_time +=  (service_1_end_time - service_1_start_time);
        syslog(LOG_CRIT, "Service 1 %llu [%lf][%lf] time taken:%lf milliseconds\n", S1Cnt,service_1_start_time-sequencer_start_time, service_1_end_time-sequencer_start_time,service_1_end_time-service_1_start_time);
        syslog(LOG_CRIT,"Service 1 %llu Jitter[%lf]",S1Cnt,frame_jitter[S1Cnt]);
        syslog(LOG_CRIT, "Service 1 total Jitter: %lf", total_jitter);
        syslog(LOG_CRIT, "Service 1 total time: %lf", service_1_total_time);  
    }
        

    service_1_average_execution_time = (service_1_total_time)/NUM_CYCLES;
    printf("Service 1 average execution time: %lf", service_1_average_execution_time);
    printf("Service 1 total jitter: %lf", total_jitter);
    syslog(LOG_CRIT,"Service 1 average execution time: %lf", service_1_average_execution_time);
    syslog(LOG_CRIT,"Service 1 total jitter: %lf", total_jitter);
    pthread_exit((void *)0);

}

void *Service_2(void *threadp)
{
    struct timeval current_time_val;
    double current_time;
    unsigned long long S2Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Service 2  @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Service 2 thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    while(!abortS2)
    {
        sem_wait(&semS2);
        S2Cnt++;
        service_2_start_time = getTimeMsec();
        best_effort_custom_sharpen_frame(frame);
        //sharpen_frame();
        //custom_sharpen();
        service_2_end_time= getTimeMsec();
        syslog(LOG_CRIT, "Sharpen Image Service  %llu [%lf][%lf] time taken:%lf milliseconds\n",S2Cnt,service_2_start_time-sequencer_start_time, service_2_end_time-sequencer_start_time,service_2_end_time-service_2_start_time);
    }

    pthread_exit((void *)0);
}

void capture_frame(){
    
    before_capture = getTimeMsec();
    capture.read(frame);
    if(frame.empty()){
        syslog(LOG_CRIT,"Frame has been missed!");
        return;
    }
    after_capture = getTimeMsec();
    syslog(LOG_CRIT,"\nframe_count %d",frame_count);
    image_object.frame = frame;
    image_object.time_stamp = getTimeMsec();
    image_buffer.push(image_object);
    
        frame_count++;
        
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

void *grab_frame(void *threadp){
    while(1){
empty_queue: 
        if(image_buffer.empty()){
            //syslog(LOG_CRIT,"frame grabbed waiting in queue");
            goto empty_queue;
        }
        image_t grabbed_image_object = image_buffer.front();
        putText(frame, to_string(getTimeMsec()), cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
        putText(frame, "Abikamet Nathan", cvPoint(30,450), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
        sprintf(grabbed_image_name,"grabbed frames/grabbed_image_%d.ppm",frame_count);
        imwrite(grabbed_image_name,grabbed_image_object.frame);
        if(run_sobel){
        Mat sobel_transformed_frame = sobel_transform(grabbed_image_object.frame);
        sprintf(grabbed_image_name,"transformed frames/sobel_transformed_image_%d.pgm",frame_count);
        imwrite(grabbed_image_name,sobel_transformed_frame);
        }
        image_buffer.pop();

    }
}

void RT_threads_setup(){
     printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

   CPU_ZERO(&allcpuset);

   for(i=0; i < NUM_CPU_CORES; i++)
       CPU_SET(i, &allcpuset);

   printf("Using CPUS=%d from total available.\n", CPU_COUNT(&allcpuset));


    // initialize the sequencer semaphores
    if (sem_init (&semS1, 0, 1)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&semS2, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    

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
      //rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

      rt_param[i].sched_priority=rt_max_prio-i;
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);
      threadParams[i].threadIdx=i;
    }
   
    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));
}

void best_effort_thread_setup(){

       int rc;
       CPU_ZERO(&best_effort_thread_cpu);
       CPU_SET(0, &best_effort_thread_cpu);
       CPU_SET(1, &best_effort_thread_cpu);
       CPU_SET(2, &best_effort_thread_cpu);
       rc=pthread_attr_init(&best_effort_attr[0]);
       rc=pthread_attr_setinheritsched(&best_effort_attr[0], PTHREAD_EXPLICIT_SCHED);
       rc=pthread_attr_setschedpolicy(&best_effort_attr[0], SCHED_FIFO);
       rc=pthread_attr_setaffinity_np(&best_effort_attr[0], sizeof(cpu_set_t), &best_effort_thread_cpu);
       pthread_attr_setschedparam(&best_effort_attr[0], &best_effort_param[0]);
       printf("\nBest Effort threads setup completed");
}

void camera_setup(){
    if(!capture.isOpened()){
      printf("\nCamera could not be found!");
    }
    capture.set(CV_CAP_PROP_FRAME_WIDTH,HRES);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,VRES);
}

void update_current_deadline(){
  #if TEN_HZ
  if(periodic_interval_duration.tv_nsec == 900000000){
     periodic_interval_duration.tv_nsec = 0;
     periodic_interval_duration.tv_sec+=1;
     syslog(LOG_CRIT,"periodic_interval_time second = %d",periodic_interval_duration.tv_sec);
  }else{
      periodic_interval_duration.tv_nsec += 100000000;

  }
  syslog(LOG_CRIT,"periodic_interval_time nano sec = %lld\n",periodic_interval_duration.tv_nsec);
  #else
  periodic_interval_duration.tv_sec+=1;
  //periodic_interval_duration.tv_sec=0;
  //periodic_interval_duration.tv_nsec += 999999999;
  #endif
  current_deadline.tv_sec = start_time.tv_sec + periodic_interval_duration.tv_sec;  
  current_deadline.tv_nsec = start_time.tv_nsec + periodic_interval_duration.tv_nsec;
}

double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};
  clock_gettime(CLOCK_REALTIME, &event_ts);
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

void sharpen_frame(){
    GaussianBlur(frame, frame, Size(0, 0), 3);
    addWeighted(frame, 1.5, frame, -0.5, 0, frame);
    sprintf(image_name,"sharpened_image_%d.ppm",frame_count);
    imwrite(image_name,frame);
}

void best_effort_custom_sharpen_frame(Mat mat_frame){
    int x,y;
    double temp;
    for(x=1;x<frame.rows+1;x++){
        for(y=1;y<frame.cols+1;y++){
            for(int channel=0;channel<3;channel++){
                temp = 0;
                temp += (double)PSF[0]*frame.at<Vec3b>(x-1,y-1)[channel];
                temp += (double)PSF[1]*frame.at<Vec3b>(x-1,y)[channel];
                temp += (double)PSF[2]*frame.at<Vec3b>(x-1,y+1)[channel];
                temp += (double)PSF[3]*frame.at<Vec3b>(x,y-1)[channel];
                temp += (double)PSF[4]*frame.at<Vec3b>(x,y)[channel];
                temp += (double)PSF[5]*frame.at<Vec3b>(x,y+1)[channel];
                temp += (double)PSF[6]*frame.at<Vec3b>(x+1,y-1)[channel];
                temp += (double)PSF[7]*frame.at<Vec3b>(x+1,y)[channel];
                temp += (double)PSF[8]*frame.at<Vec3b>(x+1,y+1)[channel];
                if(temp<0.0) temp=0.0;
	            if(temp>255.0) temp=255.0;
                frame.at<Vec3b>(x,y)[channel]=temp;
            }
        }
    }
    sprintf(image_name,"custom_sharpened_image_%d.ppm",frame_count);
    printf("Mat type number %d",frame.type());
    imwrite(image_name,frame);  
    return;
}

void custom_sharpen(){
    int x,y;
    double temp;
    for(x=1;x<frame.rows+1;x++){
        for(y=1;y<frame.cols+1;y++){
            for(int channel=0;channel<3;channel++){
                temp = 0;
                temp += (double)PSF[0]*frame.at<Vec3b>(x-1,y-1)[channel];
                temp += (double)PSF[1]*frame.at<Vec3b>(x-1,y)[channel];
                temp += (double)PSF[2]*frame.at<Vec3b>(x-1,y+1)[channel];
                temp += (double)PSF[3]*frame.at<Vec3b>(x,y-1)[channel];
                temp += (double)PSF[4]*frame.at<Vec3b>(x,y)[channel];
                temp += (double)PSF[5]*frame.at<Vec3b>(x,y+1)[channel];
                temp += (double)PSF[6]*frame.at<Vec3b>(x+1,y-1)[channel];
                temp += (double)PSF[7]*frame.at<Vec3b>(x+1,y)[channel];
                temp += (double)PSF[8]*frame.at<Vec3b>(x+1,y+1)[channel];
                if(temp<0.0) temp=0.0;
	            if(temp>255.0) temp=255.0;
                frame.at<Vec3b>(x,y)[channel]=temp;
            }
        }
    }
    sprintf(image_name,"custom_sharpened_image_%d.ppm",frame_count);
    printf("Mat type number %d",frame.type());
    imwrite(image_name,frame);   
}


