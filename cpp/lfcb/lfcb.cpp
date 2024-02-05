#include <iostream> // for print function
#include <cstring>  // mem functions
#include <atomic>   // for atomic variables
#include "lfcb.h"

/**
 * @struct LFCB structure that includes fields related to lock-free circular buffer
 */
struct LFCB {
  //! head that points to front of buffer
  std::atomic<int> head;
  //! tail that points to rear of buffer
  std::atomic<int> tail;
  //! buffer size to prevent possible overflow
  std::atomic<int> size;
  //! buffer content
  int buf[LFCB_BUFFER_SIZE];
};

//! local lock-free circular buffer instance
static LFCB lfcb;

int lfcb_init(void) {
  lfcb.head.store(0);
  lfcb.tail.store(0);
  lfcb.size.store(0);
  
  // zeroize content of circular buffer
  memset(lfcb.buf, 0x00, sizeof(int) * LFCB_BUFFER_SIZE);
  
  return 0;
}

int lfcb_enqueue(int elem) {
  const int size = lfcb.size.load();
  
  // if the buffer is full, there is no place for enqueuing
  if (LFCB_BUFFER_SIZE == size) {
    return -1;
  }

  const int tail = lfcb.tail.load();
  
  lfcb.buf[tail] = elem;
  lfcb.tail.store((tail + 1) % LFCB_BUFFER_SIZE);
  lfcb.size.store(size + 1);
  
  return 0;
}

int lfcb_dequeue(int &elem) {
  const int size = lfcb.size.load();
  
  // if the buffer is 0, there is no element for dequeuing
  if (0 == size) {
    return -1;
  }

  const int head = lfcb.head.load();
  
  elem = lfcb.buf[head];
  lfcb.head.store((head + 1) % LFCB_BUFFER_SIZE);
  lfcb.size.store(size - 1);
  
  return 0;
}

void lfcb_print(void) {
  int begin = lfcb.head.load();
  int end = lfcb.tail.load();

  if (0 == lfcb.size.load()) {
    return;
  }

  if (end > begin) {
    for (int i = begin; i < end; ++i) {
      std::cout << "[" << lfcb.buf[i] << "]" << "->";
    }
  }
  // if tail points smaller index than head points, that means buffer was circulated
  else {
    for (int i = end; i < LFCB_BUFFER_SIZE; ++i) {
      std::cout << "[" << lfcb.buf[i] << "]" << "->";
    }

    for (int i = 0; i < begin; ++i) {
      std::cout << "[" << lfcb.buf[i] << "]" << "->";
    }
  }

  std::cout << std::endl;
  
  return;
}
