#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <semaphore.h>
#include <errno.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>
#include <mqueue.h>

#define SNDRCV_MQ "/posix_mq"
#define MAX_MSG_SIZE 128
#define ERROR -1

struct mq_attr mq_attr2;
pthread_t receiver_thread,sender_thread;
pthread_attr_t thread_attribute;

static char canned_msg[] = "this is a test, and only a test, in the event of a real emergency, you would be instructed ...";

void receiver(void)
{
  printf("\n \n Entered receiver");
  mqd_t mymq;
  char buffer[MAX_MSG_SIZE];
  int prio;
  int nbytes;

  mymq = mq_open(SNDRCV_MQ, O_CREAT|O_RDWR, 0664, &mq_attr2);

  if(mymq == (mqd_t)ERROR)
    perror("mq_open");

  /* read oldest, highest priority msg from the message queue */
  if((nbytes = mq_receive(mymq, buffer, MAX_MSG_SIZE, &prio)) == ERROR)
  {
    perror("mq_receive");
  }
  else
  {
    buffer[nbytes] = '\0';
    printf("\nreceive: msg %s received with priority = %d, length = %d",
           buffer, prio, nbytes);
  }
    
}


void sender(void)
{
  printf("\n \n Entered sender");
  mqd_t mymq;
  int prio;
  int nbytes;
 //printf("\nsender:crossed mymq declaration");

  mymq = mq_open(SNDRCV_MQ, O_RDWR, 0664, NULL);
  
  if(mymq == (mqd_t)ERROR){
    printf("\nsender mq open error");
    perror("mq_open");
  }

  /* send message with priority=30 */
  if((nbytes = mq_send(mymq, canned_msg, sizeof(canned_msg), 30)) == ERROR)
  {
    perror("mq_send");
  }
  else
  {
    printf("\nsend: message successfully sent");
  }
  
}


int main(){

      mq_attr2.mq_maxmsg = 100;
      mq_attr2.mq_msgsize = MAX_MSG_SIZE;
      mq_attr2.mq_flags = 0;
      mq_attr2.mq_curmsgs =0;

      int i=0;
     while(i<10){

      pthread_create(&receiver_thread,NULL,receiver,NULL);
      pthread_create(&sender_thread,NULL,sender,NULL);

      pthread_join(receiver_thread,NULL);
      pthread_join(sender_thread,NULL);
      i++;
   /* setup common message q attributes */
     }
   return 0;
}



   


