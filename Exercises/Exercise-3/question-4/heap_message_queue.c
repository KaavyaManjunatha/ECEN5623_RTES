/****************************************************************************/
/*                                                                          */
/* Sam Siewert - 10/14/97                                                   */
/*                                                                          */
/*                                                                          */
/****************************************************************************/                                                                  
#include "mqueue.h"
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

#define SNDRCV_MQ "/heap_mq"
#define ERROR -1

struct mq_attr mq_attr2;
pthread_t receiver_thread,sender_thread;
pthread_attr_t thread_attribute;

struct mq_attr mq_attr;
static mqd_t mymq;

/* receives pointer to heap, reads it, and deallocate heap memory */

void receiver(void)
{
  char buffer[sizeof(void *)+sizeof(int)];
  void *buffptr; 
  int prio;
  int nbytes;
  int count = 0;
  int id;
  int i=0;
  while(i<10) {

    /* read oldest, highest priority msg from the message queue */

    printf("Reading %ld bytes\n", sizeof(void *));
  
    if((nbytes = mq_receive(mymq, buffer, (size_t)(sizeof(void *)+sizeof(int)), &prio)) == ERROR)
/*
    if((nbytes = mq_receive(mymq, (void *)&buffptr, (size_t)sizeof(void *), &prio)) == ERROR)
*/
    {
      perror("mq_receive");
    }
    else
    {
      memcpy(&buffptr, buffer, sizeof(void *));
      memcpy((void *)&id, &(buffer[sizeof(void *)]), sizeof(int));
      printf("receive: ptr msg 0x%X received with priority = %d, length = %d, id = %d\n", buffptr, prio, nbytes, id);

      printf("contents of ptr = \n%s\n", (char *)buffptr);

      free(buffptr);

      printf("heap space memory freed\n");
    }
    i++;
    
  }

}


static char imagebuff[4096];

void sender(void)
{
  char buffer[sizeof(void *)+sizeof(int)];
  void *buffptr;
  int prio;
  int nbytes;
  int id = 999;
  int i=0;

  while(i<10) {

    /* send malloc'd message with priority=30 */

    buffptr = (void *)malloc(sizeof(imagebuff));
    strcpy(buffptr, imagebuff);
    printf("Message to send = %s\n", (char *)buffptr);

    printf("Sending %ld bytes\n", sizeof(buffptr));

    memcpy(buffer, &buffptr, sizeof(void *));
    memcpy(&(buffer[sizeof(void *)]), (void *)&id, sizeof(int));

    if((nbytes = mq_send(mymq, buffer, (size_t)(sizeof(void *)+sizeof(int)), 30)) == ERROR)
    {
      perror("mq_send");
    }
    else
    {
      printf("send: message ptr 0x%X successfully sent\n", buffptr);
    }
    i++;
    usleep(3000000);

  }
  
}



static int sid, rid;

int main(void)
{
  int i, j;
  char pixel = 'A';

  for(i=0;i<4096;i+=64) {
    pixel = 'A';
    for(j=i;j<i+64;j++) {
      imagebuff[j] = (char)pixel++;
    }
    imagebuff[j-1] = '\n';
  }
  imagebuff[4095] = '\0';
  imagebuff[63] = '\0';

  printf("buffer =\n%s", imagebuff);

  /* setup common message q attributes */
  mq_attr2.mq_maxmsg = 100;
  mq_attr2.mq_msgsize = sizeof(void *)+sizeof(int);
  mq_attr2.mq_curmsgs =0;
  mq_attr2.mq_flags = 0;

  
  mymq = mq_open(SNDRCV_MQ, O_CREAT|O_RDWR, 0644, &mq_attr2);
 

  if(mymq == (mqd_t)ERROR){
    perror("mq_open");
    exit(1);
  }
  
   printf("\nmessage queue opened successfully\n");

   pthread_create(&receiver_thread,NULL,receiver,NULL);
   pthread_create(&sender_thread,NULL,sender,NULL);

   pthread_join(receiver_thread,NULL);
   pthread_join(sender_thread,NULL);

   mq_close(mymq);
   mq_unlink(mymq);
}
