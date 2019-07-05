#include <pthread.h>
#include <stdio.h>
#include <sched.h>
#include <time.h>
#include <stdlib.h>



int randomnumber = 0;
struct timespec time_state = {0,0};


double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};

  clock_gettime(CLOCK_MONOTONIC, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}

int main(){
    printf("\n******Random number check*********\n");
    int odd_count=0,even_count=0;
    int odd_wins=0,even_wins=0;
    int i=0,j=0;
    srand(getTimeMsec());
    while(j<100){
        srand(getTimeMsec());
        odd_count=0;
        even_count=0;
        while(i<100){
            randomnumber = (int)(getTimeMsec()*rand())%2;
            if(randomnumber){
                odd_count++;
            }else{
                even_count++;
            }
            i++;
        }
        if(odd_count>even_count){
            odd_wins++;
            //printf("\nodd won");
        }else if(odd_count<even_count){
            even_wins++;
            //printf("\neven won");
        }else{
            odd_wins++;
            even_wins++;
        }
        j++;
    }

    printf("\n odd count=%d \n even count=%d\n",odd_wins,even_wins);
        return 0;
}