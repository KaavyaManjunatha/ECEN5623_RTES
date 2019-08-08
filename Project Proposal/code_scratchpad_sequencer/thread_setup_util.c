#include "main.h"

extern cpu_set_t threadcpu;
extern pthread_t threads[NUM_THREADS];
extern threadParams_t threadParams[NUM_THREADS];
extern pthread_attr_t rt_sched_attr[NUM_THREADS];
extern uint8_t rt_max_prio, rt_min_prio;

extern struct sched_param rt_param[NUM_THREADS];
extern struct sched_param main_param;
extern pthread_attr_t main_attr;
extern pid_t mainpid;
extern cpu_set_t allcpuset;
extern sem_t sem_capture;

extern pthread_t best_effort_threads[NUM_BEST_EFFORT_THREADS];
extern pthread_attr_t best_effort_attr[NUM_BEST_EFFORT_THREADS];
extern struct sched_param best_effort_param[NUM_BEST_EFFORT_THREADS];
extern cpu_set_t best_effort_thread_cpu;


extern typedef struct
{
    int threadIdx;
    int image_height;
    int image_width;
} threadParams_t;



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