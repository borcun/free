#include <iostream>
#include <pthread.h>
#include <cstdlib>
#include <ctime>
#include "lfcb.h"

/**
 * @brief function that produces data for the buffer
 * @param [in] arg - enqueuing count
 * @return NULL
 */
void *produce(void *arg);

/**
 * @brief function that consumes data from the buffer
 * @param [in] arg - dequeuing count
 * @return NULL
 */
void *consume(void *arg);

int main() {
  // consumer and producer threads
  pthread_t producer;
  pthread_t consumer;
  // arbitrary values for enqueuing and dequeuing
  int enqCount = 2021;
  int deqCount = 2008;

  // for random numbers to be enqueued
  srand(time(NULL));

  std::cout << "Consumer-Producer competion starts..." << std::endl;
  
  pthread_create(&producer, NULL, produce, (void *) (&enqCount));
  pthread_create(&consumer, NULL, consume, (void *) (&deqCount));

  pthread_join(producer, NULL);
  pthread_join(consumer, NULL);

  std::cout << "The competion is done." << std::endl;
  std::cout << "Queue Content after the competion: " << std::endl;

  /*
   * the buffer may include some elements, be full or be empty. this situation is normal,
   * because we do not know how many enqueuing tries are performed when the buffer is full,
   * or dequeuing tries when the buffer is empty.
   */
  lfcb_print();
  
  return 0;
}

void *produce(void *arg) {
  int count = *((int *) arg);

  while (count > 0) {
    // generate random number in arbitrary range, then enqueue it
    lfcb_enqueue(rand() % 100);
    --count;
  }

  pthread_exit(NULL);
  
  return NULL;
}

void *consume(void *arg) {
  int count = *((int *) arg);
  int elem = 0;
  
  while (count > 0) {  
    lfcb_dequeue(elem);    
    --count;
  }

  pthread_exit(NULL);
  
  return NULL;
}
