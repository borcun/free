#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include "cbufTest.h"

//! default start offset to start counting
#define DEFAULT_NUMBERING_RANGE (1000)
//! enqueue operation count for testing purposes
#define DEFAULT_ENQUEUE_COUNT (2020)
//! dequeue operation count for testing purposes
#define DEFAULT_DEQUEUE_COUNT (2022)

/**
 * @brief function that shows usage of application
 */
void usage(void);

/**
 * @brief function that executes concurrent operation on circular buffer
 * @param [in] enqCount - enqueue count
 * @param [in] deqCount - dequeu count
 * @return 0 if execution is done successfully. otherwise, return error code.
 */
int executeConcurrently(int enqCount, int deqCount);

/**
 * @brief function that performs enqueuing operation on circular buffer
 * @param [in] arg - generic argument
 * @return NULL
 */
void *enqueue(void *arg);

/**
 * @brief function that performs dequeuing operation on circular buffer
 * @param [in] arg - generic argument
 * @return NULL
 */
void *dequeue(void *arg);

int main(int argc, const char * argv[]) {
#if CBUF_TEST
   cbufTest_init();
   cbufTest_transaction();
   cbufTest_deinit();
#else
   int enqCount = DEFAULT_ENQUEUE_COUNT;
   int deqCount = DEFAULT_DEQUEUE_COUNT;
   
   // application accepts argument number as 1 or 3
   if (1 != argc) {
      if (3 == argc) {
         enqCount = atoi(argv[1]);
         deqCount = atoi(argv[2]);
         
         if (enqCount < 0 || deqCount < 0) {
            printf("The count values must be positive\n");
            usage();
            return -1;
         }
      }
      else {
         usage();
         return -1;
      }
   }

   // to get random numbers for enqueuing
   srand((unsigned int) time(NULL));
   
   if (0 != executeConcurrently(enqCount, deqCount)) {
      perror("executeConcurrently");
      return -1;
   }
#endif

   return 0;
}

void usage(void) {
   printf("./<executable>\n");
   printf(" or\n");
   printf("./<executable> <enqueue count> <dequeue count>\n");
   printf("note: enqueue and dequeue operation counts are %d %d respectively\n",
          DEFAULT_ENQUEUE_COUNT,
          DEFAULT_DEQUEUE_COUNT);
   
   return;
}

void *enqueue(void *arg) {
   int enqCount = *((int *) arg);
   int elem = 0;
   
   /*
    * generally, a professional condition is provided for condition of 'while' loop,
    * e.g. it may be 'true' for an infinite operation, or breakable condition from thread loop.
    * the below condition, decreasing counter, is thoroughly demo-purpose.
    */
   while (0 != enqCount) {
      // get number between [0, 999) to be enqueued randomly
      elem = rand() % DEFAULT_NUMBERING_RANGE;
      
      cbuf_enqueue(elem); // no need to check return value here
      --enqCount;

      /*
       * printf is intentionally added since it makes a bit latency. this thread function is
       * demo-purpose. in a solution as operational, no need printf or random number generator
       */
      printf("+(%d) ", elem);
   }
   
   return NULL;
}

void *dequeue(void *arg) {
   int deqCount = *((int *) arg);
   int elem = 0;
   
   while (0 != deqCount) {
      cbuf_dequeue(&elem); // no need to check return value here
      --deqCount;

      // printf is intentionally added since it makes a bit latency
      printf("-(%d) ", elem);
   }
   
   return NULL;
}

int executeConcurrently(int enqCount, int deqCount) {
   pthread_t enqThread;
   pthread_t deqThread;
   int err = 0;
   
   if (0 != (err = cbuf_init())) {
      return err;
   }
   
   pthread_create(&enqThread, NULL, enqueue, &enqCount);
   pthread_create(&deqThread, NULL, dequeue, &deqCount);
   
   pthread_join(enqThread, NULL);
   pthread_join(deqThread, NULL);

   printf("\n\nCircular Buffer Final Content: \n");
   cbuf_print();

   if (0 != (err = cbuf_deinit())) {
      return err;
   }
   
   return 0;
}
