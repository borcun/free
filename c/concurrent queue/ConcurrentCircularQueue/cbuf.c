#include <string.h>
#include <pthread.h>
#include "cbuf.h"

/**
 * @struct CircularBuffer that is internal structure for circular buffer definition
 */
typedef struct {
   //! head index
   int head;

   //! tail index
   int tail;

   //! data of circular buffer
   int data[CBUF_DEFAULT_BUF_SIZE];

} CircularBuffer;

//! private instance of circular buffer
static CircularBuffer cBuf = {.head = 0, .tail = 0};

//! mutex to prevent parallel enqueue and dequeue operations, and provide concurrent operations
static pthread_mutex_t cbMutex;

int cbuf_init(void) {
   int err = 0;

   if (0 != (err = pthread_mutex_init(&cbMutex, NULL))) {
      perror("circular buffer mutex");
      return err;
   }

   /*
    * set all fields of circular buffer to zero, including data field though head and tail were
    * already set to zero. if tail has not made circulated yet, and dequeuing is done more than
    * enqueuing, the user encounters default data value, namely 0 when dequeued.
    */
   memset(&cBuf, 0x00, sizeof(CircularBuffer));

   return 0;
}

int cbuf_deinit(void) {
   memset(&cBuf, 0x00, sizeof(CircularBuffer));

   return pthread_mutex_destroy(&cbMutex);
}

int cbuf_enqueue(int elem) {
   int err = 0;
   
   if (0 != (err = pthread_mutex_lock(&cbMutex))) {
      perror("enqueue lock");
      return err;
   }
   
   // put element into location indicated by tail
   cBuf.data[cBuf.tail] = elem;
   cBuf.tail++;
   // circulate tail if buffer exceed, no need to check whether tail reaches to size of buffer
   cBuf.tail %= CBUF_DEFAULT_BUF_SIZE;
   
   if (0 != (err = pthread_mutex_unlock(&cbMutex))) {
      perror("enqueue unlock");
      return err;
   }
   
   return 0;
}

int cbuf_dequeue(int *elem) {
   int err = 0;
   
   // do not process more if pointer is null
   if (NULL == elem) {
      // more meaningful error code may be returned
      return -1;
   }
   
   if (0 != (err = pthread_mutex_lock(&cbMutex))) {
      perror("dequeue lock");
      return err;
   }
   
   // get element from index indicated by head
   *elem = cBuf.data[cBuf.head];
   cBuf.head++;
   // circulate head if exceed
   cBuf.head %= CBUF_DEFAULT_BUF_SIZE;
   
   if (0 != (err = pthread_mutex_unlock(&cbMutex))) {
      perror("dequeue lock");
      return err;
   }
   
   return 0;
}

void cbuf_print(void) {
   int i = 0;
   
   for (i = 0; i < CBUF_DEFAULT_BUF_SIZE; ++i) {
      if (i == cBuf.head - 1 && i == cBuf.tail - 1) {
         printf("HT>%d ", cBuf.data[i]);
      }
      else if (i == cBuf.head - 1) {
         printf("H>%d ", cBuf.data[i]);
      }
      else if (i == cBuf.tail - 1) {
         printf("T>%d ", cBuf.data[i]);
      }
      else {
         printf("%d ", cBuf.data[i]);
      }
   }
   
   printf("\n");
   
   return;
}
